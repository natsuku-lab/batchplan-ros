//! Collision ownership without changing the URDF's kinematic frames.

use std::collections::{HashMap, HashSet};
use std::ops::Range;

use anyhow::{Context, Result, ensure};
use roxmltree::Document;

/// A URDF divided between the robot and the world it is fixed in.
pub(crate) struct Partition {
    /// The URDF with collision geometry on the robot's links only.
    pub robot_urdf: String,
    /// The robot's links; `None` when every link is.
    pub robot_links: Option<HashSet<String>>,
}

impl Partition {
    pub fn new(urdf: &str, roots: &[String]) -> Result<Self> {
        if roots.is_empty() {
            return Ok(Self {
                robot_urdf: urdf.to_string(),
                robot_links: None,
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
        let world_collisions = doc
            .root_element()
            .children()
            .filter(|n| n.has_tag_name("link"))
            .filter(|n| !n.attribute("name").is_some_and(|l| selected.contains(l)))
            .flat_map(|n| n.children().filter(|c| c.has_tag_name("collision")))
            .map(|c| c.range())
            .collect();
        Ok(Self {
            robot_urdf: without(urdf, world_collisions),
            robot_links: Some(selected.into_iter().map(str::to_string).collect()),
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

#[cfg(test)]
mod tests {
    use super::*;

    const CELL: &str = r#"<robot name="cell">
      <link name="base"><collision><geometry><box size="1 1 1"/></geometry></collision>
        <visual><geometry><box size="1 1 1"/></geometry></visual></link>
      <link name="spacer"/>
      <link name="arm"><collision><geometry><sphere radius="0.1"/></geometry></collision></link>
      <link name="tool"><collision><geometry><sphere radius="0.1"/></geometry></collision></link>
      <link name="wall"><collision><geometry><box size="1 1 1"/></geometry></collision></link>
      <joint name="mount" type="fixed"><parent link="base"/><child link="spacer"/></joint>
      <joint name="axis" type="revolute"><parent link="spacer"/><child link="arm"/></joint>
      <joint name="tip" type="fixed"><parent link="arm"/><child link="tool"/></joint>
      <joint name="fixture" type="fixed"><parent link="base"/><child link="wall"/></joint>
    </robot>"#;

    #[test]
    fn selection_keeps_kinematics_and_removes_world_collisions() {
        let partition = Partition::new(CELL, &["spacer".into()]).unwrap();
        assert_eq!(
            partition.robot_links.unwrap(),
            HashSet::from(["arm".into(), "tool".into()])
        );
        let doc = Document::parse(&partition.robot_urdf).unwrap();
        assert_eq!(
            doc.descendants()
                .filter(|n| n.has_tag_name("joint"))
                .count(),
            4
        );
        assert_eq!(
            doc.descendants()
                .filter(|n| n.has_tag_name("collision"))
                .count(),
            2
        );
    }

    #[test]
    fn roots_are_strict_and_world_joints_must_be_fixed() {
        let all = Partition::new(CELL, &[]).unwrap();
        assert_eq!(all.robot_urdf, CELL);
        assert!(all.robot_links.is_none());
        assert!(Partition::new(CELL, &["missing".into()]).is_err());
        assert!(Partition::new(CELL, &["tool".into()]).is_err());
        assert!(Partition::new(CELL, &["arm".into()]).is_err());
        let selected = Partition::new(CELL, &["base".into(), "spacer".into()]).unwrap();
        assert!(!selected.robot_links.unwrap().contains("base"));
    }
}
