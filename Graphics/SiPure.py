import numpy as np
import matplotlib.pyplot as plt
import serial
import serial.tools.list_ports
import random
import time

# Configuration
LightCubeSizes = [[8, 8, 32], [16, 16, 16], [24, 24, 32]]
FRAME_RATE = 30  # frames per second
BEAM_RADIUS = 2
BEAM_INTERVAL = 2

# Initialize global variables
cubePort = None
LightCubeOld = None
fig = None
ax = None
scatter = None

def fill_bottom_levels(light_cube, levels=8, green_ratio=0.92, white_ratio=0.05, blue_ratio=0.03):
    total_leds = levels * light_cube.shape[1] * light_cube.shape[2]
    num_green = int(total_leds * green_ratio)
    num_white = int(total_leds * white_ratio)
    num_blue = total_leds - num_green - num_white

    flat_cube = light_cube[:, :, :levels].reshape(-1, 3)
    flat_cube[:num_green, 1] = 1
    flat_cube[num_green:num_green + num_white, :] = 1
    flat_cube[num_green + num_white:num_green + num_white + num_blue, 2] = 1

    np.random.shuffle(flat_cube)
    light_cube[:, :, :levels] = flat_cube.reshape((light_cube.shape[0], light_cube.shape[1], levels, 3))

def remove_led(light_cube, x, y, z, *channels):
    speed = 0.2
    angle = random.uniform(0, 2 * np.pi)
    velocity = np.array([np.cos(angle), np.sin(angle), random.uniform(0.5, 1.5)]) * speed

    position = np.array([x, y, z], dtype=float)

    while all(0 <= pos < light_cube.shape[i] for i, pos in enumerate(position)):
        light_cube[int(position[0]), int(position[1]), int(position[2]), :] = 0
        position += velocity
        if all(0 <= pos < light_cube.shape[i] for i, pos in enumerate(position)):
            light_cube[int(position[0]), int(position[1]), int(position[2]), list(channels)] = 1

    # Replace with green LED if in the bottom layers
    if z < 8:
        light_cube[x, y, z, 1] = 1

def shoot_beam(light_cube, radius, interval):
    cube_size = light_cube.shape[0]
    for x in range(0, cube_size, interval):
        for y in range(0, cube_size, interval):
            for i in range(cube_size):
                for j in range(cube_size):
                    if (i - x) ** 2 + (j - y) ** 2 <= radius ** 2:
                        for k in range(light_cube.shape[2]):
                            if light_cube[i, j, k, 1] == 1:
                                remove_led(light_cube, i, j, k, 1)
                            elif light_cube[i, j, k, 0] == 1 and light_cube[i, j, k, 2] == 1:
                                remove_led(light_cube, i, j, k, 0, 2)
                            elif light_cube[i, j, k, 2] == 1:
                                remove_led(light_cube, i, j, k, 2)

def update_display_and_cube(light_cube):
    global cubePort, LightCubeOld, fig, ax, scatter

    if LightCubeOld is None:
        LightCubeOld = np.copy(light_cube)
        if fig is None:
            fig = plt.figure()
            ax = fig.add_subplot(111, projection='3d')
            scatter = ax.scatter([], [], [], c=[])

    packet = getUpdatedVoxels(light_cube, LightCubeOld)
    LightCubeOld = np.copy(light_cube)
    if cubePort is not None:
        cubePort.write(bytearray(packet, 'utf-8'))
    else:
        update_scatter_plot(ax, scatter, light_cube)

def update_scatter_plot(ax, scatter, light_cube):
    colors = []
    positions = []

    for i in range(light_cube.shape[0]):
        for j in range(light_cube.shape[1]):
            for k in range(light_cube.shape[2]):
                if light_cube[i, j, k, 0]:
                    colors.append('red')
                    positions.append((i, j, k))
                elif light_cube[i, j, k, 1]:
                    colors.append('green')
                    positions.append((i, j, k))
                elif light_cube[i, j, k, 2]:
                    colors.append('blue')
                    positions.append((i, j, k))

    if positions:
        positions = np.array(positions)
        scatter._offsets3d = (positions[:, 0], positions[:, 1], positions[:, 2])
        scatter.set_color(colors)
        ax.set_xlim(0, light_cube.shape[0])
        ax.set_ylim(0, light_cube.shape[1])
        ax.set_zlim(0, light_cube.shape[2])
    ax.figure.canvas.draw_idle()

def getUpdatedVoxels(NewFrame, OldFrame):
    Diff = np.bitwise_xor(NewFrame, OldFrame)
    DiffPos = np.nonzero(Diff)
    UpdatePacket = ""

    i_last = -1
    j_last = -1
    k_last = -1

    for Pn in range(len(DiffPos[0])):
        i = DiffPos[0][Pn]
        j = DiffPos[1][Pn]
        k = DiffPos[2][Pn]

        if i_last == i and j_last == j and k_last == k:
            continue
        else:
            i_last = i
            j_last = j
            k_last = k
            UpdatePacket += chr(0b01000000 | NewFrame[i, j, k, 0] << 5 | i)
            UpdatePacket += chr(0b00000000 | NewFrame[i, j, k, 1] << 5 | j)
            UpdatePacket += chr(0b00000000 | NewFrame[i, j, k, 2] << 5 | k)

    return UpdatePacket

def main():
    global cubePort, LightN, LightCube, FRAME_RATE

    try:
        Ports = serial.tools.list_ports.comports()
        if len(Ports) == 0:
            raise
        print(*Ports)
        CommPortID = int(input("Select Port: "))
        cubePort = serial.Serial("COM" + str(CommPortID), 115200)
        print("connected to cube")
    except:
        cubePort = None
        print("Could not connect to THE CUBE")

    try:
        sizeSelection = int(input("Select Size: 1:{8,8,32}, 2:{16,16,16}, 3:{24,24,32} : "))
        LightN = LightCubeSizes[sizeSelection - 1]
        LightCube = np.zeros([LightN[0], LightN[1], LightN[2], 3], dtype="bool")
    except:
        print("Failed to select a cube size")
        raise

    fill_bottom_levels(LightCube, 8, 0.92, 0.05, 0.03)

    last_time = time.time()
    frame_duration = 1.0 / FRAME_RATE

    while True:
        current_time = time.time()
        elapsed_time = current_time - last_time
        if elapsed_time >= frame_duration:
            shoot_beam(LightCube, BEAM_RADIUS, BEAM_INTERVAL)
            update_display_and_cube(LightCube)
            last_time = current_time

if __name__ == "__main__":
    main()
    if cubePort is None:
        plt.show()
