#!/usr/bin/env python3
"""Generate Gazebo collisions from the official-demo obstacle layout."""

import math
import sys
import xml.etree.ElementTree as ET

from forest_layout import PILLARS, RINGS, RING_TUBE_RADIUS, ring_centerline


def add_cylinder(link, name, pose, radius, length, color):
    # The upstream random_forest environment is a planning point cloud, not a
    # contact-physics model. Gazebo renders the same geometry for the video.
    for kind in ("visual",):
        element = ET.SubElement(link, kind, {"name": f"{kind}_{name}"})
        ET.SubElement(element, "pose").text = " ".join(f"{value:.5f}" for value in pose)
        geometry = ET.SubElement(element, "geometry")
        cylinder = ET.SubElement(geometry, "cylinder")
        ET.SubElement(cylinder, "radius").text = f"{radius:.5f}"
        ET.SubElement(cylinder, "length").text = f"{length:.5f}"
        if kind == "visual":
            material = ET.SubElement(element, "material")
            ET.SubElement(material, "ambient").text = color
            ET.SubElement(material, "diffuse").text = color


def segment_pose(start, end):
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    dz = end[2] - start[2]
    horizontal = math.hypot(dx, dy)
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    return (
        (start[0] + end[0]) / 2.0,
        (start[1] + end[1]) / 2.0,
        (start[2] + end[2]) / 2.0,
        0.0,
        math.atan2(horizontal, dz),
        math.atan2(dy, dx),
    ), length


def main(path):
    tree = ET.parse(path)
    root = tree.getroot()
    world = root.find("world")
    forest = next(model for model in world.findall("model")
                  if model.get("name") == "forest")
    link = forest.find("link")
    for child in list(link):
        link.remove(child)

    for index, (x, y, radius, height) in enumerate(PILLARS):
        add_cylinder(link, f"pillar_{index}",
                     (x, y, height / 2.0, 0.0, 0.0, 0.0),
                     radius, height, "0.38 0.40 0.42 1")

    for ring_index, ring in enumerate(RINGS):
        points = ring_centerline(ring, segments=20)
        for segment_index, start in enumerate(points):
            end = points[(segment_index + 1) % len(points)]
            pose, length = segment_pose(start, end)
            add_cylinder(link, f"ring_{ring_index}_{segment_index}", pose,
                         RING_TUBE_RADIUS, length, "0.62 0.64 0.68 1")

    ET.indent(tree, space="  ")
    tree.write(path, encoding="UTF-8", xml_declaration=True)


if __name__ == "__main__":
    main(sys.argv[1])
