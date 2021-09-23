from bpy import context
from bpy import path
from bpy import ops
from bpy import app
from bpy import data
from math import pi

import sys
argv = sys.argv
argv = argv[argv.index("--") + 1:]  # get all args after "--"

print(app.version_string)
blenderVersion = app.version
blenderVersionChar = app.version_char
blenderBuildBranch = app.build_branch

scene = context.scene
scene.render.image_settings.file_format = 'PNG'
scene.render.image_settings.compression = 25
sceneName = path.basename(context.blend_data.filepath)

scene.render.resolution_percentage = 40

meshName = 'gpiow_book00'

if meshName == 'gpiow_book01':
    scene.render.resolution_x = 1080
    scene.render.resolution_y = 1920
    active_cam = context.scene.camera
    data.cameras[active_cam.name].lens = 16
    data.objects[active_cam.name].rotation_euler[0] = 100 * pi / 180

print('Scene: {}'.format(sceneName))
print('Resolution: {} x {}'.format(scene.render.resolution_x, scene.render.resolution_y))
print('Resolution Percentage: {}'.format(scene.render.resolution_percentage))

for iFrame in range(512):
    ops.import_mesh.ply(filepath="C:\\Users\\...\\Documents\\github\\gpiow\\build\\"+meshName+"_"+'{0:06d}'.format(iFrame)+".ply")
    context.scene.render.filepath = "C:\\Users\\...\\Documents\\github\\gpiow\\build\\"+meshName+"_"+'{0:06d}'.format(iFrame)
    ops.render.render(write_still=True)

    objs = data.objects
    objs.remove(objs[meshName+"_"+'{0:06d}'.format(iFrame)], do_unlink=True)

print('DONE')
