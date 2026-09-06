import numpy as np
import json
from Robots.class_Robot_Roki_2 import Robot
import os
from pathlib import Path

cwd = os.getcwd()
p = Path(cwd + "\\Soccer\\Motion\\motion_slots\\test")
fileList = list(p.glob('**/*.json')) 

robot = Robot()
for file in fileList:
    filename = file.name
    dirname = file.parent
    full_filename = str(dirname) + '\\' + filename
    with open(full_filename, 'r') as f:
        aDict = json.loads(f.read())
    motionName = filename[:-5]
    motionList = aDict[motionName]

    cppText = "// This is motion '" + motionName + "' translated from json file \n\n"
    cppText += "#include <roki2met.h>\n"
    cppText += "int frameCount;\n\n"
    cppText += "void main(){\n\n"
    try:
        pages = aDict["pageNames"]
    except Exception:
        pages = []
        for i in range(len(motionList)):
            pages.append('page '+ str(i))

    for i in range( len(pages)):
        cppText += "// " + pages[i] + '\n'
        cppText += "  frameCount = " + str(motionList[i][0]) + ";\n"
        for j in range(len(motionList[i])-1):
            rokiPosition = int(motionList[i][j+1] * 1.536 *  robot.ACTIVESERVOS[j][7])
            if i > 0:
                if rokiPosition == int(motionList[i-1][j+1] * 1.536 *  robot.ACTIVESERVOS[j][7]): continue
            cppText += "  sfPoseGroup( " + robot.ACTIVESERVOS[j][6] + ", " + str(rokiPosition) + ", frameCount );\n"
            
        cppText += "  sfWaitFrame( frameCount );\n"

    cppText += "}\n"
    filename = filename[:-5] + ".cpp"
    full_filename = str(dirname) + '\\' + filename
    with open(full_filename, 'w') as f_out:
        print(cppText, file = f_out)
    print(filename, ' translated')

print(cppText)