//! Collision ownership without changing the URDF's kinematic frames.

use std::collections::{HashMap, HashSet};
use std::ops::Range;

use anyhow::{Context, Result, ensure};
use roxmltree::Document;

pub(crate) struct Partition {
    pub robot_urdf: String,
    pub robot_links: Option<HashSet<String>>,
    pub exemptions: Vec<(String, String)>,
}

impl Partition {
    pub fn new(urdf: &str, roots: &[String]) -> Result<Self> {
        if roots.is_empty() {
            return Ok(Self {
                robot_urdf: urdf.to_string(),
                robot_links: None,
                exemptions: vec![],
            });
        }
        let doc = Document::parse(urdf).context("partitioning the workcell URDF")?;
        let links: HashSet<_> = doc
            .root_element()
            .children()
            .filter(|n| n.has_tag_name("link"))
            .filter_map(|n| n.attribute("name"))
            .collect();
        let mut children = HashMap::<&str, Vec<&str>>::new();
        let mut parents = HashMap::new();
        for joint in doc
            .root_element()
            .children()
            .filter(|n| n.has_tag_name("joint"))
        {
            let link = |tag: &str| {
                joint
                    .children()
                    .find(|n| n.has_tag_name(tag))
                    .and_then(|n| n.attribute("link"))
                    .with_context(|| format!("joint lacks {tag}"))
            };
            let parent = link("parent")?;
            let child = link("child")?;
            ensure!(
                links.contains(parent) && links.contains(child),
                "joint references an unknown link"
            );
            ensure!(
                parents
                    .insert(child, (parent, joint.attribute("type") == Some("fixed")))
                    .is_none(),
                "link '{child}' has multiple parent joints"
            );
            children.entry(parent).or_default().push(child);
        }
        let mut selected = HashSet::new();
        let mut pending = vec![];
        for root in roots {
            ensure!(
                links.contains(root.as_str()),
                "unknown robot_link_roots link '{root}'"
            );
            pending.extend(children.get(root.as_str()).into_iter().flatten().copied());
        }
        while let Some(link) = pending.pop() {
            if selected.insert(link) {
                pending.extend(children.get(link).into_iter().flatten().copied());
            }
        }
        ensure!(
            !selected.is_empty(),
            "robot_link_roots selects no descendant links"
        );
        for (child, (_, fixed)) in &parents {
            ensure!(
                selected.contains(child) || *fixed,
                "world link '{child}' has a non-fixed parent joint; select its parent subtree in robot_link_roots"
            );
        }
        let mut remove_robot = vec![];
        let mut geometry = HashSet::new();
        for link in doc
            .root_element()
            .children()
            .filter(|n| n.has_tag_name("link"))
        {
            let name = link.attribute("name").context("link lacks name")?;
            if !selected.contains(name) {
                // BatchPlan falls back to visuals when a link has no collision elements.
                remove_robot.extend(
                    link.children()
                        .filter(|n| n.has_tag_name("visual"))
                        .map(|n| n.range()),
                );
            }
            for collision in link.children().filter(|n| n.has_tag_name("collision")) {
                geometry.insert(name);
                if !selected.contains(name) {
                    remove_robot.push(collision.range());
                }
            }
        }
        let mut components = HashMap::new();
        for &link in &links {
            let mut ancestor = link;
            let mut seen = HashSet::new();
            while let Some(&(parent, true)) = parents.get(ancestor) {
                ensure!(seen.insert(ancestor), "URDF contains a fixed-joint cycle");
                ancestor = parent;
            }
            components.insert(link, ancestor);
        }
        let mut exemptions = vec![];
        for &robot in geometry.iter().filter(|l| selected.contains(*l)) {
            for &world in geometry.iter().filter(|l| !selected.contains(*l)) {
                if components[robot] == components[world] {
                    exemptions.push((robot.to_string(), world.to_string()));
                }
            }
            let mut ancestor = robot;
            let mut seen = HashSet::new();
            while let Some(&(parent, _)) = parents.get(ancestor) {
                ensure!(seen.insert(ancestor), "URDF contains a joint cycle");
                if geometry.contains(parent) {
                    if !selected.contains(parent) {
                        exemptions.push((robot.to_string(), parent.to_string()));
                    }
                    break;
                }
                ancestor = parent;
            }
        }
        Ok(Self {
            robot_urdf: without(urdf, remove_robot),
            robot_links: Some(selected.into_iter().map(str::to_string).collect()),
            exemptions,
        })
    }
}

fn without(text: &str, mut ranges: Vec<Range<usize>>) -> String {
    ranges.sort_by_key(|r| r.start);
    let mut out = String::new();
    let mut start = 0;
    for range in ranges {
        out.push_str(&text[start..range.start]);
        start = range.end;
    }
    out.push_str(&text[start..]);
    out
}
