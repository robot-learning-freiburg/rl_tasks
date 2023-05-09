import iai_bullet_sim as ibs


sim = ibs.BasicSimulator(step_frequency=30, real_time=True)

sim.init('gui')

dw = sim.load_urdf('package://rl_sim_tasks/objects/dishwasher/dw.urdf', useFixedBase=True)

cubes = []
for x in range(2):
    for y in range(10):
        cubes.append(sim.create_cylinder(0.1, 0.01, ibs.Transform.from_xyz_rpy(-0.2 + 0.32* x, -0.2 + 0.05 * y, 0.4, 1.4, 0, 0)))

while True:
    sim.update()
