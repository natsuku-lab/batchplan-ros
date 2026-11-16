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
    fn selection_keeps_kinematics_and_removes_world_collision_and_visual_fallback() {
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
        assert!(!doc.descendants().any(|n| n.has_tag_name("visual")));
        assert!(
            partition
                .exemptions
                .contains(&("arm".into(), "base".into()))
        );
        assert!(
            !partition
                .exemptions
                .contains(&("arm".into(), "wall".into()))
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
