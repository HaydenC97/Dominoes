#Function for testing importing MatLab function outputs to Python.
#DE3 Robotics Dominoes Group
#Dyson School of Design Engineering, Imperial College London
#13th March 2019
import matlab.engine
eng = matlab.engine.connect_matlab() #Connects to open MatLab session.
#Open MatLab function and run matlab.engine.shareEngine from MatLab command window
res = eng.IKfunction(0,0,0,0,0,0,1.571,0,-0.5,0.17) #Call fucntion with desired start joint angles and target position
print(res[0]) #print result
