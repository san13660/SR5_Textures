# Christopher Sandoval 13660
# 03/12/2019
# SR5: Textures

from bitmap import Bitmap
from obj import Obj
from mtl import Mtl
from collections import namedtuple
from random import randint
from texture import Texture

V2 = namedtuple('Vertex2',['x', 'y'])
V3 = namedtuple('Vertex3',['x', 'y', 'z'])

# Producto punto de vectores
def dot(v0, v1):
    return v0.x * v1.x + v0.y * v1.y + v0.z * v1.z

# Producto cruz de vectores
def cross(v0, v1):
    return V3(
        v0.y * v1.z - v0.z * v1.y,
        v0.z * v1.x - v0.x * v1.z,
        v0.x * v1.y - v0.y * v1.x
    )

# Encontrar el vector unitario
def unitary(v):
    l = length(v)

    if not l:
        return V3(0, 0, 0)

    return V3(v.x/l, v.y/l, v.z/l)

# Suma de vectores
def sum(v0, v1):
    return V3(v0.x + v1.x, v0.y + v1.y, v0.z + v1.z)

# Resta de vectores
def sub(v0, v1):
    return V3(v0.x - v1.x, v0.y - v1.y, v0.z - v1.z)

# Multiplicacion de vectores
def mul(v, c):
    return V3(v.x * c, v.y * c, v.z * c)

# Longitud del vector
def length(v):
    return (v.x**2 + v.y**2 + v.z**2)**0.5

class SoftwareRenderer(object):
    def __init__(self):
        self.bitmap = None

    # Inicializa el objeto bitmap
    def glInit(self):
        self.bitmap = Bitmap()

    # Sirve para definir el tamaño d la imagen
    def glCreateWindow(self, width, height):
        self.bitmap.createWindow(width, height)

    # Sirve para definir el area en la que se desea dibujar
    def glViewPort(self, x, y, width, height):
        self.bitmap.setViewPort(x, y, width, height)

    # Sirve para definir el color con el que se limpia la ventana
    def glClearColor(self, r,g,b):
        r=int(r*255)
        g=int(g*255)
        b=int(b*255)
        self.bitmap.setClearColor(r,g,b)

    # Sirve para definir el color del vertex
    def glColor(self, r,g,b):
        r=int(r*255)
        g=int(g*255)
        b=int(b*255)
        self.bitmap.setVertexColor(r,g,b)

    # Sirve para definir la posición de un punto
    def glVertex(self, x, y):
        newX = int(self.bitmap.viewPortX + ((self.bitmap.viewPortWidth)*((x+1)/2)))
        newY = int(self.bitmap.viewPortY + ((self.bitmap.viewPortHeigth)*((y+1)/2)))
        self.bitmap.drawPoint(newX, newY)

    # Sirve para definir la posición absoluta de un punto
    def glVertexAbs(self, x, y):
        self.bitmap.drawPoint(x, y)

    # Sirve para limpiar toda la imagen con un color
    def glClear(self):
        self.bitmap.clearWindow()

    # Sirve para generar el archivo
    def glFinish(self, filename):
        self.bitmap.write(filename)

    # Sirve para dibujar una linea con posiciones absolutas
    def glLineAbsPos(self, x1, y1, x2, y2):

        # La distancia entre la posicion inicial y la final
        dy = abs(y2 - y1)
        dx = abs(x2 - x1)

        # Nos indica si el avance en y va a depender del avance en x o viceversa
        steep = dy > dx

        # Si la inclinacion es mayor a 45 grados entonces el avance en x dependera de y, se invierten las variables x y y
        if steep:
            tempX1 = x1
            x1 = y1
            y1 = tempX1

            tempX2 = x2
            x2 = y2
            y2 = tempX2

        # Si la linea va de derecha a izquierda se invierten las variables de x/y inicial y final para que siga funcionando el algoritmo
        if x1 > x2:
            tempX1 = x1
            x1 = x2
            x2 = tempX1

            tempY1 = y1
            y1 = y2
            y2 = tempY1

        # La distancia entre la posicion inicial y la final
        dy = abs(y2 - y1)
        dx = abs(x2 - x1)
        
        # El offset de la posicion y con respecto a la posicion inicial
        offset = 0

        # El limite que debe sobrepasar la posicion y para que se avance un pixel en y. Lo iniciamos en dx ya que cada ciclo de x le sumaremos 2*dy al offset y lo compararemos con 2*dx. Es otra forma de hacer lo siguiente pero SIN introducir decimales: iniciar el offset en 0, cada ciclo sumarle dx y compararlo con 0.5dy.
        threshold = dx

        # La variable y empieza en la y inicial
        y = y1

        # Se recorren todos los pixeles en x
        for x in range(x1, x2 + 1):
            # Si la linea tenia una pendiente mayor a 45 grados entonces se invirtieron las variables, por lo que se llama al comando con x y y invertidas
            if steep:
                self.bitmap.drawPoint(y, x)
            else:
                self.bitmap.drawPoint(x, y)
            
            # Se le suma al offset dos veces la diferencia de distancias en y 
            offset += dy * 2
            
            # Si el offset supera al threshold establecido entonces se mueve 1 en y
            if offset >= threshold:
                if y1 < y2:
                    y += 1
                else:
                    y -= 1

                # El treshold se ajusta para cuando toque moverse otro pixel en y sumandole dos veces dx
                threshold += dx * 2

    # Sirve para dibujar una linea con posiciones relativas (-1,1)
    def glLine(self, x1, y1, x2, y2):
        x1 = int(self.bitmap.viewPortWidth * ((x1+1)/2))
        y1 = int(self.bitmap.viewPortHeigth * ((y1+1)/2))
        x2 = int(self.bitmap.viewPortWidth * ((x2+1)/2))
        y2 = int(self.bitmap.viewPortHeigth * ((y2+1)/2))
        self.glLineAbsPos(x1, y1, x2, y2)

    # Sirve para cargar archivos .obj solo lineas
    def glLoadObjWireFrame(self, filename, translateX, translateY, scaleX, scaleY):

        # Se instancia la clase Obj
        obj = Obj(filename)

        for face in obj.faces:
            # Se establece la cantidad de vertices en cada cara
            vertexCount = len(face)

            for i in range(vertexCount):
                # Para hacer una linea se toma el vertice que indica i y el siguiente despues de i
                f1 = face[i] - 1
                nextVertex = i + 1

                # Si i es el ultimo vertice se conectara con el primero
                if nextVertex >= vertexCount:
                    nextVertex -= vertexCount

                f2 = face[nextVertex] - 1

                # Se encuentran las coordenadas de los vertices de la linea
                v1 = obj.vertices[f1]
                v2 = obj.vertices[f2]

                # Se establecen los puntos inicial y final de la linea, se aplica la traslacion y la escala
                x1 = (v1[0] + translateX) * scaleX
                y1 = (v1[1] + translateY) * scaleY
                x2 = (v2[0] + translateX) * scaleX
                y2 = (v2[1] + translateY) * scaleY

                # Se dibuja la linea
                self.glLine(x1, y1, x2, y2)

    # Sirve para cargar archivos .obj con las caras pintadas
    def glLoadObjSolid(self, filename, filenameMTL, translateX, translateY, scaleX, scaleY):

        # Se instancia la clase Obj
        obj = Obj(filename)

        mtl = Mtl(filenameMTL)

        # Direccion de la luz
        light = V3(0,0,1)

        # Se hace un contador para saber en que cara vamos para despues asignar los materiales
        faceCount = 0

        #Se crea el color default
        color = [1,1,1]

        for face in obj.faces:
            # Se obtienen las coordenadas de los tres vertices de cada cara
            x1 = int(self.bitmap.viewPortWidth * ((obj.vertices[face[0]-1][0]+1)/2))
            y1 = int(self.bitmap.viewPortWidth * ((obj.vertices[face[0]-1][1]+1)/2))
            z1 = int(self.bitmap.viewPortWidth * ((obj.vertices[face[0]-1][2]+1)/2))
            x2 = int(self.bitmap.viewPortWidth * ((obj.vertices[face[1]-1][0]+1)/2))
            y2 = int(self.bitmap.viewPortWidth * ((obj.vertices[face[1]-1][1]+1)/2))
            z2 = int(self.bitmap.viewPortWidth * ((obj.vertices[face[1]-1][2]+1)/2))
            x3 = int(self.bitmap.viewPortWidth * ((obj.vertices[face[2]-1][0]+1)/2))
            y3 = int(self.bitmap.viewPortWidth * ((obj.vertices[face[2]-1][1]+1)/2))
            z3 = int(self.bitmap.viewPortWidth * ((obj.vertices[face[2]-1][2]+1)/2))

            # Se colocan las coordenadas de los vertices en vectores
            v1 = V3(x1, y1, z1)
            v2 = V3(x2, y2, z2)
            v3 = V3(x3, y3, z3)

            # Se calcula la normal de la cara
            normal = unitary(cross(sub(v2, v1), sub(v3, v1)))
            # Se calcula el producto punto de la normal con un vector (0,0,1) para obtener el valor de intensidad de la luz
            intensity = abs(dot(normal, light))

            # Para asignar el nuevo material en la lista miramos si ya pasamos el index que nos indica un cambio de material
            if(len(obj.materialIndex) > 0 and faceCount >= obj.materialIndex[0]):
                # Se busca el nombre del material en la lista de materiales y luego se obtiene el RGB de ese material
                color = mtl.materials[obj.materialNames[0]]

                # Se eliminan esos materiales de la lista para que la siguiente vez que toque asignar material se asigne el siguiente de la lista
                obj.materialNames.pop(0)
                obj.materialIndex.pop(0)
            
            # Se le suma uno al contador de las caras
            faceCount += 1

            # Si la intensidad es negativa quiere decir que la cara da para el otro lado, por lo que no se debe dibujar
            if intensity < 0:
                continue
            
            #self.glColor((255-randint(0,255))/255, (255-randint(0,255))/255, (255-randint(0,255))/255)

            # Se configura el color de la cara
            self.glColor(color[0]*intensity, color[1]*intensity, color[2]*intensity)

            # Se dibuja la cara y se rellena
            self.glFillTriangleBarycentric(v1,v2,v3)

    # Sirve para cargar archivos .obj con las caras pintadas
    def glLoadObjTexture(self, filename, filenameT, translateX, translateY, scaleX, scaleY):

        # Se instancia la clase Obj
        obj = Obj(filename)

        texture = Texture(filenameT)

        # Direccion de la luz
        light = V3(0,0,1)

        # Se hace un contador para saber en que cara vamos para despues asignar los materiales
        faceCount = 0

        #Se crea el color default
        color = [1,1,1]

        for face in obj.faces:

            # Se obtienen las coordenadas de los tres vertices de cada cara
            x1 = int(self.bitmap.viewPortWidth * ((obj.vertices[face[0][0]-1][0]+1)/2))
            y1 = int(self.bitmap.viewPortWidth * ((obj.vertices[face[0][0]-1][1]+1)/2))
            z1 = int(self.bitmap.viewPortWidth * ((obj.vertices[face[0][0]-1][2]+1)/2))
            x2 = int(self.bitmap.viewPortWidth * ((obj.vertices[face[1][0]-1][0]+1)/2))
            y2 = int(self.bitmap.viewPortWidth * ((obj.vertices[face[1][0]-1][1]+1)/2))
            z2 = int(self.bitmap.viewPortWidth * ((obj.vertices[face[1][0]-1][2]+1)/2))
            x3 = int(self.bitmap.viewPortWidth * ((obj.vertices[face[2][0]-1][0]+1)/2))
            y3 = int(self.bitmap.viewPortWidth * ((obj.vertices[face[2][0]-1][1]+1)/2))
            z3 = int(self.bitmap.viewPortWidth * ((obj.vertices[face[2][0]-1][2]+1)/2))

            # Se colocan las coordenadas de los vertices en vectores
            v1 = V3(x1, y1, z1)
            v2 = V3(x2, y2, z2)
            v3 = V3(x3, y3, z3)

            # Se obtienen las coordenadas de los vertices de la textura
            xt1 = int(texture.width * obj.tVertices[face[0][1]-1][0]) - 1
            yt1 = int(texture.width * obj.tVertices[face[0][1]-1][1]) - 1
            xt2 = int(texture.width * obj.tVertices[face[1][1]-1][0]) - 1
            yt2 = int(texture.width * obj.tVertices[face[1][1]-1][1]) - 1
            xt3 = int(texture.width * obj.tVertices[face[2][1]-1][0]) - 1
            yt3 = int(texture.width * obj.tVertices[face[2][1]-1][1]) - 1

            # Se colocan las coordenadas de los vertices de textura en vectores
            vt1 = V3(xt1, yt1, 0)
            vt2 = V3(xt2, yt2, 0)
            vt3 = V3(xt3, yt3, 0)

            # Se calcula la normal de la cara
            normal = unitary(cross(sub(v2, v1), sub(v3, v1)))
            # Se calcula el producto punto de la normal con un vector (0,0,1) para obtener el valor de intensidad de la luz
            intensity = abs(dot(normal, light))
            
            # Se le suma uno al contador de las caras
            faceCount += 1

            # Si la intensidad es negativa quiere decir que la cara da para el otro lado, por lo que no se debe dibujar
            if intensity < 0:
                continue
            
            #self.glColor((255-randint(0,255))/255, (255-randint(0,255))/255, (255-randint(0,255))/255)

            # Se configura el color de la cara
            #self.glColor(color[0]*intensity, color[1]*intensity, color[2]*intensity)

            # Se dibuja la cara y se rellena
            self.glFillTriangleBarycentricTexture(v1,v2,v3, texture, texture_vertex=(vt1,vt2,vt3), intensity=intensity)

    # Sirve para rellenar poligonos previamente dibujados
    def glFillPolygons(self):
        # Se establece un "color" para marcar las orillas, se usan numeros negativos para asegurarse que no se confunda con ningun otro color
        edgeColor = [-1, -1, -1]

        # Se convierten todas las orillas de los poligonos al color establecido arriba
        for y in range(self.bitmap.height):
            for x in range(self.bitmap.width):
                if self.bitmap.framebuffer[y][x] != self.bitmap.clearColor:
                    self.bitmap.framebuffer[y][x] = edgeColor

        # Se recorren todos los pixeles de el archivo de abajo para arriba y de izquierda a derecha
        for y in range(self.bitmap.height):
            # Esta varible  indica si el pixel esta adentro de algun poligono
            inside = False
            
            # Estas dos variables nos ayudan a determinar si se esta entrando o saliendo de un poligono
            edgeAbove = False
            edgeBelow = False
            for x in range(self.bitmap.width):
                # Si el pixel es del color de las orillas se inicia el proceso de verificacion para saber si se esta entrando o saliendo de un poligono
                if self.bitmap.framebuffer[y][x] == edgeColor:
                    for a in range(-1,2):
                        # Para determinar si se esta entrando o saliendo de un poligono se revisa que alguno de los tres pixeles adyacentes en la linea de arriba tambien sea una orilla
                        if self.bitmap.framebuffer[y+1][x+a] == edgeColor:
                            edgeAbove = True
                        # Para determinar si se esta entrando o saliendo de un poligono se revisa que alguno de los tres pixeles adyacentes en la linea de abajo tambien sea una orilla
                        if self.bitmap.framebuffer[y-1][x+a] == edgeColor:
                            edgeBelow = True
                    # Si las dos condiciones de arriba se cumplen quiere decir que se esta entrando o saliendo de un poligono
                    if edgeBelow and edgeAbove:
                        # Se invierte la variable que nos dice si esta adentro o afuera de un poligono 
                        inside = not inside
                        edgeBelow = False
                        edgeAbove = False
                else:
                    # Si actualmente se esta en un pixel que no sea una orilla y se esta adentro de un poligono entonces se debe pintar el pixel
                    if inside:
                        self.bitmap.framebuffer[y][x] = self.bitmap.vertexColor

        # Por ultimo se repintan todas las orillas de los poligonos a su color original
        for y in range(self.bitmap.height):
            for x in range(self.bitmap.width):
                if self.bitmap.framebuffer[y][x] == edgeColor:
                    self.bitmap.framebuffer[y][x] = self.bitmap.vertexColor

    # Sirve para definir la caja en la que se van a verificar las coordenadas barycentricas
    def boundingBox(self, A,B,C):
        x = sorted([A.x, B.x, C.x])
        y = sorted([A.y, B.y, C.y])
        return V2(x[0], y[0]), V2(x[2], y[2])

    # Sirve para encontrar las coordenadas barycentricas de un triangulo
    def barycentric(self, A, B, C, P):
        cx, cy, cz = cross(V3(B.x - A.x, C.x - A.x, A.x - P.x), V3(B.y - A.y, C.y - A.y, A.y - P.y))

        # Se contempla el caso en el que cz sea cero
        if cz == 0:
            w = -1
            v = -1
            u = -1
        else:
            # Se obtienen las coordenadas barycentricas
            u = cx/cz 
            v = cy/cz
            w = 1 - (u + v)
        return w, v, u

    # Se pintan los triangulos con ayuda de coordenadas barycentricas
    def glFillTriangleBarycentric(self, A, B, C):
        # Se calcula la caja en la que estara el triangulo
        bbox_min, bbox_max = self.boundingBox(A, B, C)

        # Para cada punto de la caja se calculara si se encuentra dentro del triangulo para saber si se pinta
        for x in range(bbox_min.x, bbox_max.x + 1):
            for y in range(bbox_min.y, bbox_max.y + 1):
                # Se obtienen las coordenadas barycentricas
                w, v, u = self.barycentric(A, B, C, V2(x, y))

                # Si alguna de las coordenadas es menor a cero se salta este punto ya que no esta 
                if w < 0 or v < 0 or u < 0:
                    continue

                # Se calcula el valor de z para saber donde va en el zbuffer
                z = A.z * w + B.z * v + C.z * u

                # Si la z de este punto de esta cara es mayor a la que ya estaba en esta posicion entonces se dibuja
                if z > self.bitmap.zbuffer[x][y]:
                    self.bitmap.drawPoint(x,y)
                    self.bitmap.zbuffer[x][y] = z

    # Se pintan los triangulos con ayuda de coordenadas barycentricas
    def glFillTriangleBarycentricTexture(self, A, B, C, texture, texture_vertex, intensity):
        # Se calcula la caja en la que estara el triangulo
        bbox_min, bbox_max = self.boundingBox(A, B, C)

        # Para cada punto de la caja se calculara si se encuentra dentro del triangulo para saber si se pinta
        for x in range(bbox_min.x, bbox_max.x + 1):
            for y in range(bbox_min.y, bbox_max.y + 1):
                # Se obtienen las coordenadas barycentricas
                w, v, u = self.barycentric(A, B, C, V2(x, y))

                # Si alguna de las coordenadas es menor a cero se salta este punto ya que no esta 
                if w < 0 or v < 0 or u < 0:
                    continue

                # Se calcula la x y y de la posicion en la imagen de textura
                tA, tB, tC = texture_vertex
                tx = tA.x * w + tB.x * v + tC.x * u
                ty = tA.y * w + tB.y * v + tC.y * u

                # Se obtiene el color de la textura y se calcula su intensidad
                color = texture.get_color_at_pos(tx, ty)
                color1 = (color[0] * intensity)/255.0
                color2 = (color[1] * intensity)/255.0
                color3 = (color[2] * intensity)/255.0

                # Se cambia el color
                glColor(color1,color2,color3)

                # Se calcula el valor de z para saber donde va en el zbuffer
                z = A.z * w + B.z * v + C.z * u

                # Si la z de este punto de esta cara es mayor a la que ya estaba en esta posicion entonces se dibuja
                if z > self.bitmap.zbuffer[x][y]:
                    self.bitmap.drawPoint(x,y)
                    self.bitmap.zbuffer[x][y] = z

                


# Instanciación del objeto
sr = SoftwareRenderer()

#---------------------Métodos Globales-----------------------

def glInit():
    sr.glInit()

def glCreateWindow(width, height):
    sr.glCreateWindow(width,height)

def glViewPort(x, y, width, height):
    sr.glViewPort(x,y,width,height)

def glClearColor(r,g,b):
    sr.glClearColor(r,g,b)

def glColor(r,g,b):
    sr.glColor(r,g,b)

def glVertex(x, y):
    sr.glVertex(x,y)

def glVertexAbs(x,y):
    sr.glVertexAbs(x,y)

def glClear():
    sr.glClear()

def glFinish(filename='out.bmp'):
    sr.glFinish(filename)

def glLine(x1, y1, x2, y2):
    sr.glLine(x1, y1, x2, y2)

def glLineAbsPos(x1, y1, x2, y2):
    sr.glLineAbsPos(x1, y1, x2, y2)

def glLoadObjWireFrame(filename, translateX=0, translateY=0, scaleX=1, scaleY=1):
    sr.glLoadObjWireFrame(filename, translateX, translateY, scaleX, scaleY)

def glLoadObjSolid(filename, filenameMTL, translateX=0, translateY=0, scaleX=1, scaleY=1):
    sr.glLoadObjSolid(filename, filenameMTL, translateX, translateY, scaleX, scaleY)

def glLoadObjTexture(filename, filenameT, translateX=0, translateY=0, scaleX=1, scaleY=1):
    sr.glLoadObjTexture(filename, filenameT, translateX, translateY, scaleX, scaleY)

def glFillPolygons():
    sr.glFillPolygons()

def glFillTriangleBarycentric(A, B, C):
    sr.glFillTriangleBarycentric(A, B, C)