# Christopher Sandoval 13660
# 03/12/2019
# SR5: Textures

import software_renderer as sr

sr.glInit()
sr.glCreateWindow(800,800)
sr.glViewPort(0,0,800,800)
sr.glClear()
sr.glLoadObjTexture('Bob-omb5.obj', 'textura_concreto.bmp')
sr.glFinish()