//! MoveIt's planning groups, end effectors and virtual joints, resolved against the URDF's joints.

use std::collections::{BTreeMap, HashSet};

use anyhow::{Context, Result, bail};

/// A URDF's joints by the links they connect: all MoveIt needs to resolve groups.
pub struct Tree {
    /// joint name, parent link, child link
    joints: Vec<(String, String, String)>,
}

impl Tree {
    pub fn parse(urdf: &str) -> Result<Self> {
        let doc = roxmltree::Document::parse(urdf).context("parsing the URDF")?;
        let child = |n: roxmltree::Node, tag: &str| {
            n.children().find(|c| c.has_tag_name(tag)).and_then(|c| c.attribute("link")).map(str::to_string)
        };
        let joints = doc
            .root_element()
            .children()
            .filter(|n| n.has_tag_name("joint"))
            .map(|n| {
                let name = n.attribute("name").context("a URDF joint without a name")?.to_string();
                let parent = child(n, "parent").with_context(|| format!("joint '{name}' has no parent"))?;
                let link = child(n, "child").with_context(|| format!("joint '{name}' has no child"))?;
                Ok((name, parent, link))
            })
            .collect::<Result<_>>()?;
        Ok(Self { joints })
    }

    /// The link no joint has as its child.
    pub fn root(&self) -> Option<&str> {
        let children: HashSet<&str> = self.joints.iter().map(|(_, _, c)| c.as_str()).collect();
        self.joints.iter().map(|(_, p, _)| p.as_str()).find(|p| !children.contains(p))
    }

    fn parent_joint(&self, link: &str) -> Option<&(String, String, String)> {
        self.joints.iter().find(|(_, _, c)| c == link)
    }
}

#[derive(Default)]
struct Group {
    chains: Vec<(String, String)>,
    joints: Vec<String>,
    links: Vec<String>,
    subgroups: Vec<String>,
}

#[derive(Default)]
pub struct Srdf {
    groups: BTreeMap<String, Group>,
    /// End effectors' parent links: the frames pose goals usually name.
    end_effector_links: Vec<String>,
    /// Parent frames of fixed virtual joints, which coincide with the robot's root.
    fixed_frames: Vec<String>,
}

impl Srdf {
    pub fn parse(text: &str) -> Result<Self> {
        let doc = roxmltree::Document::parse(text).context("parsing the SRDF")?;
        let attr = |n: roxmltree::Node, a: &str| {
            n.attribute(a).map(str::to_string).with_context(|| format!("SRDF <{}> without {a}", n.tag_name().name()))
        };
        let mut srdf = Srdf::default();
        for n in doc.root_element().children().filter(roxmltree::Node::is_element) {
            match n.tag_name().name() {
                "group" => {
                    let mut group = Group::default();
                    for c in n.children().filter(roxmltree::Node::is_element) {
                        match c.tag_name().name() {
                            "chain" => group.chains.push((attr(c, "base_link")?, attr(c, "tip_link")?)),
                            "joint" => group.joints.push(attr(c, "name")?),
                            "link" => group.links.push(attr(c, "name")?),
                            "group" => group.subgroups.push(attr(c, "name")?),
                            _ => {}
                        }
                    }
                    srdf.groups.insert(attr(n, "name")?, group);
                }
                "end_effector" => srdf.end_effector_links.push(attr(n, "parent_link")?),
                "virtual_joint" if n.attribute("type") == Some("fixed") => {
                    srdf.fixed_frames.push(attr(n, "parent_frame")?)
                }
                _ => {}
            }
        }
        Ok(srdf)
    }

    pub fn has_group(&self, name: &str) -> bool {
        self.groups.contains_key(name)
    }

    /// The joints `group` moves, as MoveIt resolves them: a chain's joints from its tip up to its
    /// base, each listed link's parent joint, each listed joint, and those of nested groups.
    pub fn group_joints(&self, name: &str, tree: &Tree) -> Result<Vec<String>> {
        let mut joints = vec![];
        self.collect(name, tree, &mut joints, &mut HashSet::new())?;
        Ok(joints)
    }

    fn collect(&self, name: &str, tree: &Tree, out: &mut Vec<String>, seen: &mut HashSet<String>) -> Result<()> {
        if !seen.insert(name.to_string()) {
            bail!("group '{name}' contains itself");
        }
        let group = self.groups.get(name).with_context(|| format!("unknown group '{name}'"))?;
        let mut push = |joint: &str| {
            if !out.iter().any(|j| j == joint) {
                out.push(joint.to_string());
            }
        };
        for (base, tip) in &group.chains {
            let mut link = tip.as_str();
            while link != base {
                let (joint, parent, _) = tree
                    .parent_joint(link)
                    .with_context(|| format!("chain {base} → {tip}: {base} is not above {tip}"))?;
                push(joint);
                link = parent;
            }
        }
        for link in &group.links {
            if let Some((joint, _, _)) = tree.parent_joint(link) {
                push(joint);
            }
        }
        group.joints.iter().for_each(|j| push(j));
        for sub in &group.subgroups {
            self.collect(sub, tree, out, seen)?;
        }
        Ok(())
    }

    /// Frames pose goals may name: end effectors' parent links and chain tips.
    pub fn tips(&self) -> Vec<String> {
        let mut tips = self.end_effector_links.clone();
        for (_, tip) in self.groups.values().flat_map(|g| &g.chains) {
            if !tips.contains(tip) {
                tips.push(tip.clone());
            }
        }
        tips
    }

    pub fn fixed_frames(&self) -> &[String] {
        &self.fixed_frames
    }
}
