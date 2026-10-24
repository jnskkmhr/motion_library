# Copyright (c) 2022-2026, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Create a USD variant with fixed spherical contact pads on the box-foot G1.

Dimensions, mass and friction are provisional simulation parameters, not measured
hardware specifications. Existing joint and rigid-body frames are preserved.
"""

from __future__ import annotations

import argparse
import hashlib
import os
import xml.etree.ElementTree as ET
from pathlib import Path

from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade

from isaaclab_assets import UNITREE_G1_29DOF_BOX_FOOT_CFG


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--urdf", type=Path, required=True, help="Ball-hand URDF maintained in newton-assets.")
    parser.add_argument("--mjcf", type=Path, required=True, help="Matching MJCF, including hand friction.")
    args = parser.parse_args()
    urdf, mjcf = ET.parse(args.urdf), ET.parse(args.mjcf)
    pads = {}
    for side in ("left", "right"):
        hand = urdf.find(f"link[@name='{side}_rubber_hand']")
        geom = mjcf.find(f".//body[@name='{side}_rubber_hand']/geom")
        radius = float(hand.find("collision/geometry/sphere").get("radius"))
        center = tuple(float(v) for v in hand.find("collision/origin").get("xyz").split())
        mass = float(hand.find("inertial/mass").get("value"))
        friction = float(geom.get("friction").split()[0])
        if geom.get("type") != "sphere" or float(geom.get("size")) != radius:
            raise ValueError("URDF and MJCF must describe matching spherical hand pads.")
        if tuple(float(v) for v in geom.get("pos").split()) != center:
            raise ValueError("URDF and MJCF hand centers differ.")
        if not (0 < radius < 0.2 and 0 < mass < 5 and 0 <= friction <= 5):
            raise ValueError("Invalid radius, mass or friction in source hand definitions.")
        inertia = hand.find("inertial/inertia")
        if any(float(inertia.get(k)) != 0 for k in ("ixy", "ixz", "iyz")):
            raise ValueError("This hand overlay expects principal-axis inertias.")
        if hand.find("inertial/origin").get("xyz") != hand.find("collision/origin").get("xyz"):
            raise ValueError("Expected the hand COM at the sphere center.")
        pads[side] = (radius, center, mass, friction, tuple(float(inertia.get(k)) for k in ("ixx", "iyy", "izz")))
    source = Path(UNITREE_G1_29DOF_BOX_FOOT_CFG.spawn.usd_path).resolve()
    output = args.output.resolve()
    if output == source or output.exists():
        parser.error("Output must be a new file, distinct from the source asset.")
    output.parent.mkdir(parents=True, exist_ok=True)
    original = Usd.Stage.Open(str(source))
    stage = Usd.Stage.CreateNew(str(output))
    root = stage.DefinePrim(original.GetDefaultPrim().GetPath(), "Xform")
    root.GetReferences().AddReference(os.path.relpath(source, output.parent))
    stage.SetDefaultPrim(root)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdPhysics.SetStageKilogramsPerUnit(stage, 1.0)
    stage.GetRootLayer().documentation = (
        "Hand-only overlay generated from newton-assets URDF/MJCF. Preserves body frames and all 29 joints. "
        "Mass and friction need hardware calibration; the sphere is a rigid contact approximation."
    )
    stage.GetRootLayer().customLayerData = {
        "source_urdf": args.urdf.name,
        "source_urdf_sha256": hashlib.sha256(args.urdf.read_bytes()).hexdigest(),
        "source_mjcf": args.mjcf.name,
        "source_mjcf_sha256": hashlib.sha256(args.mjcf.read_bytes()).hexdigest(),
    }
    if pads["left"][3] != pads["right"][3]:
        raise ValueError("This variant uses the same friction material on both hands.")
    material = UsdShade.Material.Define(stage, root.GetPath().AppendPath("BallPadMaterial"))
    physics_material = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
    physics_material.CreateStaticFrictionAttr(pads["left"][3])
    physics_material.CreateDynamicFrictionAttr(pads["left"][3])
    physics_material.CreateRestitutionAttr(0.0)
    shader = UsdShade.Shader.Define(stage, material.GetPath().AppendChild("Shader"))
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(0.95, 0.65, 0.05))
    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.9)
    shader.CreateOutput("surface", Sdf.ValueTypeNames.Token)
    material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
    for side in ("left", "right"):
        radius, center, pad_mass, _, inertia = pads[side]
        hand = next(p for p in stage.Traverse() if p.GetName() == f"{side}_rubber_hand")
        hand.SetInstanceable(False)
        for child in list(hand.GetChildren()):
            child.SetActive(False)
        center = Gf.Vec3f(*center)
        mass = UsdPhysics.MassAPI.Apply(hand)
        mass.CreateMassAttr(pad_mass)
        mass.CreateCenterOfMassAttr(center)
        mass.CreateDiagonalInertiaAttr(Gf.Vec3f(*inertia))
        mass.CreatePrincipalAxesAttr(Gf.Quatf(1.0))
        sphere = UsdGeom.Sphere.Define(stage, hand.GetPath().AppendChild("ball_pad"))
        sphere.CreateRadiusAttr(radius)
        sphere.CreateExtentAttr([Gf.Vec3f(-radius), Gf.Vec3f(radius)])
        sphere.AddTranslateOp().Set(Gf.Vec3d(*center))
        sphere.CreateDisplayColorAttr([Gf.Vec3f(0.95, 0.65, 0.05)])
        UsdPhysics.CollisionAPI.Apply(sphere.GetPrim()).CreateCollisionEnabledAttr(True)
        binding = UsdShade.MaterialBindingAPI.Apply(sphere.GetPrim())
        binding.Bind(material)
        binding.Bind(material, materialPurpose="physics")
    stage.GetRootLayer().Save()
    print(f"Saved {output} from {args.urdf} and {args.mjcf}")


if __name__ == "__main__":
    main()
