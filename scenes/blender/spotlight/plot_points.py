for line in open(filename):
    line = line[1:-2]
    x, y, z = line.split(',')
    x = float(x)
    y = float(y)
    z = float(z)

    bpy.ops.mesh.primitive_ico_sphere_add(radius=0.01, location=(x, y, z))

