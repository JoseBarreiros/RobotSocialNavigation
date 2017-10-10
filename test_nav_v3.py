#########################################################
#  Implementation of Social Navigation       	        #
#  Based on pedestrian model on Helbing & Molnar (1995) #
#  Jose Barreiros, Sept 2017		    		        #
#  Human Robot Interaction, Cornell University       	#
#########################################################

import numpy as np
import matplotlib.pyplot as plt
from numpy.random import randn
import matplotlib
import random
import pdb
import math as m


def f_ellipse(P,a,b,C,A):  #evaluate point P:[y,x] in function of a rotated ellispse of parameters C: center [y x] a:major axis, b:minor axis , A:angle
	f=(((P[1]-C[1])*m.cos(A)+(P[0]-C[0])*m.sin(A))**2/a**2)+(((P[1]-C[1])*m.sin(A)-(P[0]-C[0])*m.cos(A))**2/b**2)
	return f

def F_rep(R,Xi,Vi, Unk,a,b):  #return an acceleration vector inverese proportional to the distance between Xi and a point from Unk within an ellipse directed towards the motion Vi of the robot Xi with majoy axis a and minor axis b
	#R rate of repulsion
	F=[0,0] #Force y,x

	for j in range(0,len(Unk)):  #iterate through all the elements of Unk.
		Pj=Unk[j]
		#print('Pj: ',Pj)
		alpha=m.atan(Vi[0]/Vi[1])
		if (f_ellipse(Pj,a,b,Xi,alpha)<=1):   #check if Pk is within the ellipse
			#print('in')
			#calculate unit vector of Pj-Xi
			Dif=(Xi-Pj)
			d=m.sqrt(Dif[0]**2+Dif[1]**2)
			U_dif=(Dif/d) 

			#Calculate a vector Force proportional to R and inverse proportional to the distance d.
			F=F+R*(1/d)*U_dif 
		#else:
			#print('out')
	return F

def generate_UK(I,limit_min,limit_max):   #generate I UK-entities in random coordinates in the range limit_min to limit_max 
	UK_l=[]
	print("Randomly Generating N:%d Unfamiliar entities to the robot" %I)
	for i in range(0,I):
		UK_l.append([random.uniform(limit_min,limit_max),random.uniform(limit_min,limit_max)])  #generate the coordinates of an entity and append in array UK
		UK=np.array(UK_l)
	return UK

def main():

	#Set the Figure properties
	plt.ion()  #interactive/online plot.
	ax = plt.gca()  
	size=100
	ax.set_autoscale_on(True)
	ax.set_xlim([-size,size])
	ax.set_ylim([-size,size])
	point, = plt.plot([],[]) 
	point2, = plt.plot([],[]) 

	#Initial position of the robot
	Xo=[-80,-40] # Y,X
	ax.plot(Xo[1], Xo[0],marker='x', markersize=3, color="blue")
	plt.pause(0.1)

	#Goal position of the robot
	Xf=np.array([90,90])
	plt.plot(Xf[1],Xf[0],marker='x', markersize=3, color="green")

	#initialize variables
	Vo=[4,5]  #initial velocity
	Vn=np.array(Vo) #velocity in Vn-1
	Vn_1=Vn #velocity in Vn-1
	Xn=np.array(Xo) #position
	Xn_1=Xo
	N=500  #number of iterations
	T=0.5  #period of simulation
	A=7  #rate of attraction to the goal
	R=17 #rate of repulsion
	ag=0 #acceleration towards goal
	ar=0 #repulsion acceleration

	#initialize parameters of ellipse
	a=20  #major axis
	b=10  #minor axis
    
    #generate the unknown entities
	M=100   #number of unknown entities 
	UK=generate_UK(M,-size,size) #generate_UK return an array of M elements with random generated position 

	#generate the velocities for unknown entities
	V_max_UK=5 #maximum velocity of unknown entities
	V_UK=generate_UK(M,-V_max_UK,V_max_UK)  #generate_UK return an array of M elements with random generated velocity

	point, = plt.plot(Xn[1],Xn[0],marker='o', markersize=3, color="red")

	#plot all the points UK
	for j in range(0,len(UK)):
		Pj=UK[j]
		#print('Pj: ',Pj)
		point2, = plt.plot(Pj[1],Pj[0],marker='o', markersize=3, color="blue")
		plt.draw()
		plt.pause(0.1)
	#point2.remove()

	#simulate N iterations
	for i in range(0,N):   
		print("Robot move to next position")
		#print Xn: position of the robot
		print(i)   #for debugging
		#print('Xn: ')   
		#print(Xn)  

		#goal acceleration
		ag=A*np.subtract(Xf,Xn)/np.linalg.norm(np.subtract(Xf,Xn))  #acceleration (equal to goal force if mass=1) towards the goal
		#print('an: ')
		#print(ag)  #print ag: position of the robot
	    
	    #repulsion acceleration
		ar=F_rep(R,Xn,Vn,UK,a,b) #sum of repulsion acceleration (equal to repulsion force if mass=1) of every unknown identity in UK=[UK1,Uk2,...UKj] to Xn.
	    
	    #Total acceleration
		aa=ag+ar   #total acceleration = goal acceleration+ repulsion acceleration
		
	    #updated velocity
		Vn=Vn_1+aa*T  
		#print('Vn: ')
		#print(Vn)
		
		#updated position
		Xn=Xn_1+Vn_1*T+aa*T*T  
		#print('Xn: ')
		#print(Xn)

	    #plot New Position of a robot
		ax.relim()
		#point.remove() #uncoment to remove past point.  no trace mode
		point, = plt.plot(Xn[1],Xn[0],marker='o', markersize=3, color="red")

		plt.draw()
		plt.pause(0.1)
	    
	    #plot New Position of the UK entities. Comment the "for loop" to not account for kinetics of the UK entities
		#for l in range(0,len(UK)):
			
		#	Pl=UK[l]+V_UK[l]*T*1  #updated positions of UK based on velocity V_UK
		#	#point2.remove()
		#	point2, = plt.plot(Pl[1],Pl[0],marker='o', markersize=3, color="blue")
		#	plt.draw()
		#	plt.pause(0.1)
		#	UK[l]=Pl


	    #Old velocity and position update
		Vn_1=Vn  #Vn-1=Vn
		X_1=Xn   #Xn-1=Xn

	plt.ioff()  #plot interactive mode set to off
	plt.show()  #persistant plot

if __name__ == '__main__':
    main()

	