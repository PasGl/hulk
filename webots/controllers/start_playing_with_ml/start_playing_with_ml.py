from controller import Supervisor
import random
from simple_websocket_server import WebSocketServer, WebSocket
from threading import Thread

TIME_STEP = 10

supervisor = Supervisor()
chest_button_channel = supervisor.getDevice('ChestButton Channel')
scene_control_server = None
nao_node = supervisor.getFromDef("NAO")
ball_node = supervisor.getFromDef("SPLBall")
uneven_terrain_1 = supervisor.getFromDef("UnevenTerrain1")
uneven_terrain_2 = supervisor.getFromDef("UnevenTerrain2")
uneven_terrain_3 = supervisor.getFromDef("UnevenTerrain3")
uneven_terrain_4 = supervisor.getFromDef("UnevenTerrain4")
uneven_terrain_5 = supervisor.getFromDef("UnevenTerrain5")
count = 0
resetting = False
penalized = True

class SceneControl(WebSocket):
    def handle(self):
        if self.data == "reset":
            chest_button_channel.send(b'\x01')
            global penalized
            penalized = True
            global resetting 
            resetting = True
            global count
            count = 0
            ball_node.getField('translation').setSFVec3f([random.uniform(-4.2, -2.2), random.uniform(-0.5, 0.5), 0.05])
            ball_node.setVelocity([0, 0, 0])
            uneven_terrain_1.getField('randomSeed').setSFInt32(random.randrange(1000000))
            uneven_terrain_2.getField('randomSeed').setSFInt32(random.randrange(1000000))
            uneven_terrain_3.getField('randomSeed').setSFInt32(random.randrange(1000000))
            uneven_terrain_4.getField('randomSeed').setSFInt32(random.randrange(1000000))
            uneven_terrain_5.getField('randomSeed').setSFInt32(random.randrange(1000000))
            uneven_terrain_1.getField('translation').setSFVec3f([random.uniform(-4.2, -2.2), random.uniform(-2.0, -1.0), -0.01])
            uneven_terrain_2.getField('translation').setSFVec3f([random.uniform(-4.2, -2.2), random.uniform(-2.0, -1.0), -0.01])
            uneven_terrain_3.getField('translation').setSFVec3f([random.uniform(-4.2, -2.2), random.uniform(-2.0, -1.0), -0.01])
            uneven_terrain_4.getField('translation').setSFVec3f([random.uniform(-4.2, -2.2), random.uniform(-2.0, -1.0), -0.01])
            uneven_terrain_5.getField('translation').setSFVec3f([random.uniform(-4.2, -2.2), random.uniform(-2.0, -1.0), -0.01])
            uneven_terrain_1.getField('size').setSFVec3f([random.uniform(0.5, 2.0), random.uniform(0.5, 2.0), random.uniform(0.08, 0.14)])
            uneven_terrain_2.getField('size').setSFVec3f([random.uniform(0.5, 2.0), random.uniform(0.5, 2.0), random.uniform(0.08, 0.14)])
            uneven_terrain_3.getField('size').setSFVec3f([random.uniform(0.5, 2.0), random.uniform(0.5, 2.0), random.uniform(0.08, 0.14)])
            uneven_terrain_4.getField('size').setSFVec3f([random.uniform(0.5, 2.0), random.uniform(0.5, 2.0), random.uniform(0.08, 0.14)])
            uneven_terrain_5.getField('size').setSFVec3f([random.uniform(0.5, 2.0), random.uniform(0.5, 2.0), random.uniform(0.08, 0.14)])

def run_scene_control_server():
    scene_control_server = WebSocketServer("localhost", 9980, SceneControl)
    scene_control_server.serve_forever()

websocket_thread = Thread(target=run_scene_control_server)
websocket_thread.start()

while True:
    supervisor.step(TIME_STEP) # != -1:
    if not resetting and count == 20:
        chest_button_channel.send(b'\x01')
    if not resetting and count == 220:
        chest_button_channel.send(b'\x01')
    if not resetting and count == 250:
        if penalized:
            chest_button_channel.send(b'\x01')
            penalized = not penalized
    if resetting and count == 250:
            resetting = False
            nao_node.getField('translation').setSFVec3f([-3.2, -3.0, 0.333369])
            nao_node.getField('rotation').setSFRotation([0, 0, 1, 1.57079632679])
            nao_node.setVelocity([0, 0, 0])
            count = 220
    count += 1
