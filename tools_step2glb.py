import os, cascadio, trimesh

inp = r"C:\Users\yucha\Downloads\main.step"
tmp = r"C:\Users\yucha\Downloads\_main_raw.glb"
out = r"C:\Users\yucha\AppData\Roaming\AdvantageScope\userAssets\Robot_26Offseason\model.glb"

print("input STEP MB:", round(os.path.getsize(inp) / 1e6, 2))
print("converting (this can take a minute)...")
cascadio.step_to_glb(inp, tmp)
print("raw glb MB:", round(os.path.getsize(tmp) / 1e6, 2))

scene = trimesh.load(tmp)
try:
    tri = sum(len(g.faces) for g in scene.geometry.values())
except Exception:
    tri = "?"
ext = [round(float(x), 3) for x in scene.bounding_box.extents]
print("triangles:", tri, "| raw extents (model units):", ext)

mx = max(scene.bounding_box.extents)
if mx > 50:  # ~1 m robot expressed in mm -> convert to meters
    scene.apply_transform(trimesh.transformations.scale_matrix(0.001))
    print("scaled mm -> m")

ext2 = [round(float(x), 3) for x in scene.bounding_box.extents]
print("final extents (m):", ext2)

scene.export(out)
print("WROTE:", out, "|", round(os.path.getsize(out) / 1e6, 2), "MB")
