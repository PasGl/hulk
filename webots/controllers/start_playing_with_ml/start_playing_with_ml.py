from controller import Supervisor
import random
import struct
from threading import Thread
import time
import websockets.sync.server

TIME_STEP = 10

supervisor = Supervisor()
chest_button_channel = supervisor.getDevice('ChestButton Channel')
scene_control_server = None
nao_node = supervisor.getFromDef("NAO")
nao_translation = nao_node.getField('translation')
ball_node = supervisor.getFromDef("SPLBall")
ball_translation = ball_node.getField('translation')
uneven_terrain_patches = [supervisor.getFromDef("UnevenTerrain"+str(i+1)) for i in range(2)]
count = 0
resetting = False
penalized = True
paused = False
initializing = True
fps_time = time.time()
fps_counter = 0
resetting_websocket = None
step_websocket = None

def handle_commands(websocket):
    global paused
    global penalized
    global resetting
    global count
    global resetting_websocket
    global step_websocket
    global initializing
    for message in websocket:
        if message == b'0':
            chest_button_channel.send(b'\x01')
            penalized = True
            resetting = True
            count = 0
            ball_node.getField('translation').setSFVec3f([random.uniform(-3.7, -2.7), -1.5, 0.05])
            ball_node.setVelocity([0, 0, 0])
            for uneven_terrain_patch in uneven_terrain_patches:
                uneven_terrain_patch.getField('randomSeed').setSFInt32(random.randrange(1000000))
                uneven_terrain_patch.getField('translation').setSFVec3f([random.uniform(-3.7, -2.7), random.uniform(-2.5, -2.0), -0.01])
                uneven_terrain_patch.getField('size').setSFVec3f([random.uniform(0.2, 0.4), random.uniform(1.2, 2.0), random.uniform(0.03, 0.05)])
            resetting_websocket = websocket
            initializing = True
        elif message == b'1':
            nao_pos = nao_translation.getSFVec3f()
            ball_pos = ball_translation.getSFVec3f()
            translations_bin = struct.pack('%sf' % 4, *[nao_pos[0], nao_pos[1], ball_pos[0], ball_pos[1]])
            websocket.send(translations_bin)
        elif message == b'2':
            paused = True
            websocket.send("paused")
        elif message == b'3':
            paused = False
            websocket.send("unpaused")
        elif message == b'4':
            initializing = False
            step_websocket = websocket
        

class WebSocketThread(Thread):
    def __init__(self):
        Thread.__init__(self)

    def run(self):
        with websockets.sync.server.serve(handle_commands, "localhost", 9980) as server:
            server.serve_forever()

websocket_thread = WebSocketThread()
websocket_thread.start()

while True:
    if not paused:
        if initializing or step_websocket is not None:
            supervisor.step(TIME_STEP) # != -1:
            if not resetting and count == 20:
                chest_button_channel.send(b'\x01')
            if not resetting and count == 220:
                chest_button_channel.send(b'\x01')
            if not resetting and count == 250:
                if penalized:
                    chest_button_channel.send(b'\x01')
                    penalized = not penalized
                    if resetting_websocket is not None:
                        resetting_websocket.send("resetted")
                        resetting_websocket = None
                        #initializing = False
            if resetting and count == 250:
                    resetting = False
                    nao_node.getField('translation').setSFVec3f([-3.2, -3.0, 0.333369])
                    nao_node.getField('rotation').setSFRotation([0, 0, 1, 1.57079632679])
                    nao_node.setVelocity([0, 0, 0])
                    count = 220
            count += 1
            fps_counter += 1
            if step_websocket is not None:
                step_websocket.send("stepped")
                step_websocket = None
        else:
            time.sleep(0.001)
    else:
        time.sleep(0.01)

    if time.time() - fps_time > 1:
        #print("FPS:", fps_counter)
        fps_time = time.time()
        fps_counter = 0
