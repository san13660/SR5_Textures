# Christopher Sandoval 13660
# 03/12/2019
# SR5: Textures

import struct

def char(c):
    return struct.pack("=c", c.encode('ascii'))

def word(c):
    return struct.pack("=h", c)

def dword(c):
    return struct.pack("=l", c)

def color(r, g, b):
    return bytes([b, g, r])

class Bitmap(object):
    # Se inicializan los valores
    def __init__(self):
        self.width = 0
        self.height = 0
        self.viewPortWidth = 0
        self.viewPortHeigth = 0
        self.viewPortX = 0
        self.viewPortY = 0
        self.clearColor = color(0,0,0)
        self.vertexColor = color(255,255,255)
        self.framebuffer = []

    # Se define el tamaño de la ventana
    def createWindow(self, width, height):
        self.width = width
        self.height = height
        self.clearWindow()
        
        # El tamaño default del viewport es el tamaño de la ventana
        self.viewPortWidth = width - 1
        self.viewPortHeigth = height - 1
        self.viewPortX = 0
        self.viewPortY = 0

    # Se establece el tamaño del area de dibujo
    def setViewPort(self, x, y, width, height):
        self.viewPortX = x
        self.viewPortY = y
        self.viewPortWidth = width - 1
        self.viewPortHeigth = height - 1

    # Se establece el color del clear
    def setClearColor(self, r,g,b):
        self.clearColor = color(r,g,b)

    # Se establece el color del vertex
    def setVertexColor(self, r,g,b):
        self.vertexColor = color(r,g,b)

    # Sirve para dibujar un solo punto con la posición relativa
    def drawPoint(self, x, y):
        if x>=0 and x<self.width and y>=0 and y<self.height:
            self.framebuffer[y][x] = self.vertexColor

    # Sirve para limpiar con un solo color
    def clearWindow(self):
        self.framebuffer = [
            [
                self.clearColor for x in range(self.width)
            ]
            for y in range(self.height)
        ]

        self.zbuffer = [
            [-float('inf') for x in range(self.width)]
            for y in range(self.height)
        ]

    # Sirve para escribir el archivo
    def write(self, filename):
        f = open(filename, 'bw')

        f.write(char('B'))
        f.write(char('M'))
        f.write(dword(14 + 40 + self.width * self.height *3))
        f.write(dword(0))
        f.write(dword(14 + 40))

        f.write(dword(40))
        f.write(dword(self.width))
        f.write(dword(self.height))
        f.write(word(1))
        f.write(word(24))
        f.write(dword(0))
        f.write(dword(self.width * self.height * 3))
        f.write(dword(0))
        f.write(dword(0))
        f.write(dword(0))
        f.write(dword(0))

        for x in range(self.height):
            for y in range(self.width):
                f.write(self.framebuffer[x][y])

        f.close()