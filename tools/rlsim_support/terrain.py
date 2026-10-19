"""Deterministic terrain geometry, shared by SDF generation and web previews."""
import math
import random
import xml.etree.ElementTree as ET

from .catalog import keys
from .legacy import ConfigError, number, integer


def load_world(path):
    """Keep an existing SDF world intact and read its effective physics settings."""
    try:
        text = path.read_text(encoding="utf-8")
        root = ET.fromstring(text)
    except (OSError, ET.ParseError) as error:
        raise ConfigError("Invalid world file: " + str(error)) from error
    worlds = root.findall("world")
    if root.tag != "sdf" or len(worlds) != 1:
        raise ConfigError("World file must contain exactly one sdf/world")
    world = worlds[0]
    physics = world.find("physics")
    if physics is None or physics.get("type", "ode") != "ode":
        raise ConfigError("World file requires ODE physics")
    try:
        step = float(physics.findtext("max_step_size", "0.001"))
        factor = float(physics.findtext("real_time_factor", "1"))
        gravity = [float(v) for v in world.findtext("gravity", "0 0 -9.8").split()]
    except ValueError as error:
        raise ConfigError("Invalid world physics values") from error
    number(step, "world.max_step_size", positive=True)
    number(factor, "world.real_time_factor", positive=True)
    if len(gravity) != 3:
        raise ConfigError("World gravity requires three components")
    for value in gravity:
        number(value, "world.gravity")
    return text, {"step_size": step, "real_time_factor": factor, "gravity": gravity}


def geometry(generator, params, seed):
    base = {"size", "friction"}
    extras = {"flat": set(), "stairs": {"steps", "rise", "tread", "width", "start_x"},
              "ramp": {"angle_deg", "run", "width", "start_x", "thickness"},
              "rough": {"rows", "columns", "cell", "max_height", "start_x"}}
    if generator not in extras:
        raise ConfigError("Unknown terrain generator: " + str(generator))
    keys(params, base | extras[generator], context="terrain parameters")
    for name, value in params.items():
        number(value, "terrain." + name)
        if name != "start_x" and value <= 0:
            raise ConfigError("terrain." + name + " must be positive")
    if params["size"] > 1000 or params["friction"] > 10:
        raise ConfigError("Terrain size/friction exceeds supported range")
    boxes = []

    def box(name, size, position, rpy=(0, 0, 0)):
        if any(not math.isfinite(v) or abs(v) > 10000 for v in (*size, *position, *rpy)) or any(v <= 0 for v in size):
            raise ConfigError("Generated terrain geometry exceeds supported range")
        boxes.append({"name": name, "size": list(size), "position": list(position), "rpy": list(rpy), "friction": params["friction"]})

    box("ground", (params["size"], params["size"], .1), (0, 0, -.05))
    if generator == "stairs":
        steps = integer(params["steps"], "steps", positive=True)
        if steps > 100:
            raise ConfigError("At most 100 stairs supported")
        for i in range(steps):
            height = (i + 1) * params["rise"]
            box("step_" + str(i), (params["tread"], params["width"], height),
                (params["start_x"] + (i + .5) * params["tread"], 0, height / 2))
    elif generator == "ramp":
        if not 0 < params["angle_deg"] < 75:
            raise ConfigError("Ramp angle must be between 0 and 75 degrees")
        angle = math.radians(params["angle_deg"])
        height = params["run"] * math.tan(angle)
        thickness = params["thickness"]
        box("ramp", (params["run"] / math.cos(angle), params["width"], thickness),
            (params["start_x"] + params["run"] / 2 + math.sin(angle) * thickness / 2, 0,
             height / 2 - math.cos(angle) * thickness / 2), (0, -angle, 0))
    elif generator == "rough":
        rows = integer(params["rows"], "rows", positive=True)
        columns = integer(params["columns"], "columns", positive=True)
        if rows * columns > 400:
            raise ConfigError("At most 400 rough terrain cells supported")
        rng = random.Random(integer(seed, "seed"))
        for i in range(columns):
            for j in range(rows):
                height = rng.uniform(params["max_height"] * .01, params["max_height"])
                box("cell_{}_{}".format(i, j), (params["cell"], params["cell"], height),
                    (params["start_x"] + (i + .5) * params["cell"], (j - (rows - 1) / 2) * params["cell"], height / 2))
    return {"schema_version": 1, "generator": generator, "seed": seed, "boxes": boxes}


def sdf_world(terrain, physics):
    keys(physics, {"step_size", "real_time_factor", "gravity"}, context="physics")
    step = number(physics["step_size"], "physics.step_size", positive=True)
    factor = number(physics["real_time_factor"], "physics.real_time_factor", positive=True)
    if not math.isfinite(factor / step):
        raise ConfigError("Invalid physics update rate")
    gravity = physics["gravity"]
    if not isinstance(gravity, list) or len(gravity) != 3:
        raise ConfigError("gravity requires three components")
    for value in gravity:
        number(value, "gravity")
    root = ET.Element("sdf", version="1.6")
    world = ET.SubElement(root, "world", name="default")

    def text(parent, tag, value):
        ET.SubElement(parent, tag).text = value if isinstance(value, str) else " ".join("{:.12g}".format(v) for v in value)

    text(world, "gravity", gravity)
    node = ET.SubElement(world, "physics", name="physics", type="ode")
    text(node, "max_step_size", [step])
    text(node, "real_time_factor", [factor])
    text(node, "real_time_update_rate", [factor / step])
    light = ET.SubElement(world, "light", name="sun", type="directional")
    text(light, "pose", [0, 0, 10, 0, 0, 0]); text(light, "direction", [-.5, .1, -.9])
    text(light, "diffuse", [.8, .8, .8, 1]); text(light, "specular", [.2, .2, .2, 1])
    for box in terrain["boxes"]:
        model = ET.SubElement(world, "model", name=box["name"])
        text(model, "static", "true")
        text(model, "pose", box["position"] + box["rpy"])
        link = ET.SubElement(model, "link", name="link")
        for kind in ("collision", "visual"):
            part = ET.SubElement(link, kind, name=kind)
            geom = ET.SubElement(ET.SubElement(part, "geometry"), "box")
            text(geom, "size", box["size"])
            if kind == "collision":
                friction = ET.SubElement(ET.SubElement(ET.SubElement(part, "surface"), "friction"), "ode")
                text(friction, "mu", [box["friction"]]); text(friction, "mu2", [box["friction"]])
            else:
                material = ET.SubElement(part, "material")
                text(material, "ambient", [.45, .55, .65, 1]); text(material, "diffuse", [.45, .55, .65, 1])
    return ET.tostring(root, encoding="unicode") + "\n"
