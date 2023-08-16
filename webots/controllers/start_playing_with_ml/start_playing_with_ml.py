from controller import Supervisor
import random
from threading import Thread
import time
import websockets.sync.server

TIME_STEP = 10

supervisor = Supervisor()
chest_button_channel = supervisor.getDevice('ChestButton Channel')
scene_control_server = None
nao_node = supervisor.getFromDef("NAO")
ball_node = supervisor.getFromDef("SPLBall")
uneven_terrain_patches = [supervisor.getFromDef("UnevenTerrain"+str(i+1)) for i in range(5)]
count = 0
resetting = False
penalized = True
fps_time = time.time()
fps_counter = 0

def handle_commands(websocket):
    for message in websocket:
        if message == b'0':
            chest_button_channel.send(b'\x01')
            global penalized
            penalized = True
            global resetting 
            resetting = True
            global count
            count = 0
            ball_node.getField('translation').setSFVec3f([random.uniform(-4.2, -2.2), random.uniform(-0.5, 0.5), 0.05])
            ball_node.setVelocity([0, 0, 0])
            for uneven_terrain_patch in uneven_terrain_patches:
                uneven_terrain_patch.getField('randomSeed').setSFInt32(random.randrange(1000000))
                uneven_terrain_patch.getField('translation').setSFVec3f([random.uniform(-4.2, -2.2), random.uniform(-2.0, -1.0), -0.01])
                uneven_terrain_patch.getField('size').setSFVec3f([random.uniform(0.5, 2.0), random.uniform(0.5, 2.0), random.uniform(0.08, 0.14)])
            websocket.send("resetted")

class WebSocketThread(Thread):
    def __init__(self):
        Thread.__init__(self)

    def run(self):
        with websockets.sync.server.serve(handle_commands, "localhost", 9980) as server:
            server.serve_forever()

websocket_thread = WebSocketThread()
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

    fps_counter += 1
    if time.time() - fps_time > 1:
        #print("FPS:", fps_counter)
        fps_time = time.time()
        fps_counter = 0
