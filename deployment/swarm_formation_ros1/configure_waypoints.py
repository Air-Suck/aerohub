"""Configure the upstream demo by XML identity, independent of old defaults."""
import sys
import xml.etree.ElementTree as ET


def configure(path):
    tree = ET.parse(path)
    root = tree.getroot()
    includes = [item for item in root.findall("include")
                if item.get("file", "").endswith("/advanced_param.xml")]
    if len(includes) != 1:
        raise ValueError("expected one advanced_param.xml include")
    for name, value in {"point_num": "1", "point0_x": "$(arg target_x)",
                        "point0_y": "$(arg target_y)",
                        "point0_z": "$(arg target_z)"}.items():
        matches = [arg for arg in includes[0].findall("arg") if arg.get("name") == name]
        if len(matches) != 1:
            raise ValueError("expected exactly one waypoint argument: " + name)
        matches[0].set("value", value)
    tree.write(path, encoding="unicode")


if __name__ == "__main__":
    configure(sys.argv[1])
