import xml.etree.ElementTree as ET 
import mujoco as mj
import mujoco_viewer
import numpy as np
import random

iterations = random.randrange(0, 3)

mujoco = ET.Element('mujoco')

option = ET.SubElement(mujoco, 'option')
option.text = 'timestep="0.1"'

worldbody = ET.SubElement(mujoco, 'worldbody')

light = ET.SubElement(worldbody, 'light', diffuse=".5 .5 .5", pos="0 0 3", dir="0 0 1")

floor = ET.SubElement(worldbody, 'geom', name="floor", type="plane", size="2 2 0.1")

## make the body 
bodyFrame = ET.SubElement(worldbody, 'body')
freeJoint = ET.SubElement(bodyFrame, 'freejoint', name="root")
head = ET.SubElement(bodyFrame, 'geom', name="head", pos="0 0 0.4", type="box", size="0.3 0.2 0.2", rgba="1 .75 0 1")

leg1Body = ET.SubElement(bodyFrame, 'body', name="leg1")
leg1Geom = ET.SubElement(leg1Body, 'geom', pos="0.2 0.25 0.3", type="box", size="0.05 0.05 0.15")
leg1Hinge = ET.SubElement(leg1Body, 'joint', name="leg1Hinge", pos = "0 0 0.5", axis="0 1 0", range="-10 10", limited="true")

leg2Body = ET.SubElement(bodyFrame, 'body', name="leg2")
leg2Geom = ET.SubElement(leg2Body, 'geom', pos="-0.2 0.25 0.3", type="box", size="0.05 0.05 0.15")
leg2Hinge= ET.SubElement(leg2Body, 'joint', name="leg2Hinge", pos="0 0 0.5", axis="0 1 0", range="-10 10", limited="true")

leg3Body = ET.SubElement(bodyFrame, 'body', name="leg3")
leg3Geom = ET.SubElement(leg3Body, 'geom', name="leg3", pos="0.2 -0.25 0.3", type="box", size="0.05 0.05 0.15")
leg3Hinge = ET.SubElement(leg3Body, 'joint', name="leg3Hinge", pos="0 0 0.5", axis="0 1 0", range="-10 10", limited="true")

leg4Body = ET.SubElement(bodyFrame, 'body', name="leg4")
leg4Geom = ET.SubElement(leg4Body, 'geom', name="leg4", pos="-0.2 -0.25 0.3", type="box", size="0.05 0.05 0.15")
leg4Hinge = ET.SubElement(leg4Body, 'joint', name="leg4Hinge", pos="0 0 0.5", axis="0 1 0", range="-10 10", limited="true")

actuator = ET.SubElement(mujoco,'actuator')
leg1Motor = ET.SubElement(actuator, 'motor', name="leg1Hinge", gear="50", joint="leg1Hinge")
leg2Motor = ET.SubElement(actuator, 'motor', name="leg2Hinge", gear="50", joint="leg2Hinge")
leg3Motor = ET.SubElement(actuator, 'motor', name="leg3Hinge", gear="50", joint="leg3Hinge")
leg4Motor = ET.SubElement(actuator, 'motor', name="leg4Hinge", gear="50", joint="leg4Hinge")



if iterations > 0:
    body1Body = ET.SubElement(bodyFrame, 'body')
    body1Geom = ET.SubElement(body1Body, 'geom', pos="0.65 0 0.4", type="box", size="0.3 0.2 0.2", rgba="0.75 1 0 1")
    body1Hinge = ET.SubElement(body1Body, 'joint', name="body1Hinge", pos="-0.15 0 0", axis="0 0 1", range="-10 10", limited="true")

    body1Motor = ET.SubElement(actuator, 'motor', name="body1Hinge", gear="50", joint="body1Hinge")
    
    leg5Body = ET.SubElement(body1Body, 'body', name="leg5")
    leg5Geom = ET.SubElement(leg5Body, 'geom', pos="0.85 0.25 0.3", type="box", size="0.05 0.05 0.15")
    leg5Hinge = ET.SubElement(leg5Body, 'joint', name="leg5Hinge", pos = "0 0 0.5", axis="0 1 0", range="-10 10", limited="true")

    leg6Body = ET.SubElement(body1Body, 'body', name="leg6")
    leg6Geom = ET.SubElement(leg6Body, 'geom', pos=".45 0.25 0.3", type="box", size="0.05 0.05 0.15")
    leg6Hinge= ET.SubElement(leg6Body, 'joint', name="leg6Hinge", pos="0 0 0.5", axis="0 1 0", range="-10 10", limited="true")

    leg7Body = ET.SubElement(body1Body, 'body', name="leg7")
    leg7Geom = ET.SubElement(leg7Body, 'geom', pos="0.85 -0.25 0.3", type="box", size="0.05 0.05 0.15")
    leg7Hinge = ET.SubElement(leg7Body, 'joint', name="leg7Hinge", pos = "0 0 0.5", axis="0 1 0", range="-10 10", limited="true")

    leg8Body = ET.SubElement(body1Body, 'body', name="leg8")
    leg8Geom = ET.SubElement(leg8Body, 'geom', pos=".45 -0.25 0.3", type="box", size="0.05 0.05 0.15")
    leg8Hinge= ET.SubElement(leg8Body, 'joint', name="leg8Hinge", pos="0 0 0.5", axis="0 1 0", range="-10 10", limited="true")

    leg5Motor = ET.SubElement(actuator, 'motor', name="leg5Hinge", gear="50", joint="leg5Hinge")
    leg6Motor = ET.SubElement(actuator, 'motor', name="leg6Hinge", gear="50", joint="leg6Hinge")

    leg7Motor = ET.SubElement(actuator, 'motor', name="leg7Hinge", gear="50", joint="leg7Hinge")
    leg8Motor = ET.SubElement(actuator, 'motor', name="leg8Hinge", gear="50", joint="leg8Hinge")

if iterations > 1:
    body2Body = ET.SubElement(bodyFrame, 'body')
    body2Geom = ET.SubElement(body2Body, 'geom', pos="1.3 0 0.4", type="box", size="0.3 0.2 0.2", rgba="0 1 1 1")
    body2Hinge = ET.SubElement(body2Body, 'joint', name="body2Hinge", pos="-0.15 0 0", axis="0 0 1", range="-10 10", limited="true")

    body2Motor = ET.SubElement(actuator, 'motor', name="body2Hinge", gear="50", joint="body2Hinge")
    
    leg9Body = ET.SubElement(body2Body, 'body', name="leg9")
    leg9Geom = ET.SubElement(leg9Body, 'geom', pos="1.5 0.25 0.3", type="box", size="0.05 0.05 0.15")
    leg9Hinge = ET.SubElement(leg9Body, 'joint', name="leg9Hinge", pos = "0 0 0.5", axis="0 1 0", range="-10 10", limited="true")

    leg10Body = ET.SubElement(body2Body, 'body', name="leg10")
    leg10Geom = ET.SubElement(leg10Body, 'geom', pos="1.1 0.25 0.3", type="box", size="0.05 0.05 0.15")
    leg10Hinge= ET.SubElement(leg10Body, 'joint', name="leg10Hinge", pos="0 0 0.5", axis="0 1 0", range="-10 10", limited="true")

    leg11Body = ET.SubElement(body2Body, 'body', name="leg11")
    leg11Geom = ET.SubElement(leg11Body, 'geom', pos="1.5 -0.25 0.3", type="box", size="0.05 0.05 0.15")
    leg11Hinge = ET.SubElement(leg11Body, 'joint', name="leg11Hinge", pos = "0 0 0.5", axis="0 1 0", range="-10 10", limited="true")

    leg12Body = ET.SubElement(body2Body, 'body', name="leg12")
    leg12Geom = ET.SubElement(leg12Body, 'geom', pos="1.1 -0.25 0.3", type="box", size="0.05 0.05 0.15")
    leg12Hinge= ET.SubElement(leg12Body, 'joint', name="leg12Hinge", pos="0 0 0.5", axis="0 1 0", range="-10 10", limited="true")

    leg9Motor = ET.SubElement(actuator, 'motor', name="leg9Hinge", gear="50", joint="leg9Hinge")
    leg10Motor = ET.SubElement(actuator, 'motor', name="leg10Hinge", gear="50", joint="leg10Hinge")

    leg11Motor = ET.SubElement(actuator, 'motor', name="leg11Hinge", gear="50", joint="leg11Hinge")
    leg12Motor = ET.SubElement(actuator, 'motor', name="leg12Hinge", gear="50", joint="leg12Hinge")


tree = ET.ElementTree(mujoco)
tree.write('bug.xml')

if iterations == 0:
    numMotors = 4
    step = np.array([1, -1, -1, 1])

if iterations == 1:
    numMotors = 9
    step = np.array([-1, 1, -1, 1, 7, -1, 1, -1, 1])

if iterations >1:
    numMotors = 14
    step = np.array([1, -1, 1, -1, 7, -1, 1, -1, 1, -7, -1, 1, -1, 1])


model = mj.MjModel.from_xml_path('bug.xml')
data = mj.MjData(model)

viewer = mujoco_viewer.MujocoViewer(model, data)

motors = model.nu


data.ctrl[:motors] = step

for i in range(10000):
    if viewer.is_alive:
        if i%50 == 0:
            step = step*-1
        data.ctrl[:motors] = step
        mj.mj_step(model, data)
        viewer.render()
    else:
        break

viewer.close