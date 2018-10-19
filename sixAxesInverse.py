import numpy as np 
from math import *


class Inverse:
	def __init__(self,x,y,z,A,B,C,t):
		
		# self.x = 400.44;
		# self.y = -82.19;
		# self.z = 171.43;
		# self.A = 104.3/180*pi;
		# self.B = 73.35/180*pi;
		# self.C = -95.11/180*pi;

		## cartpose 
		self.x = x
		self.y = y
		self.z = z
		self.A = A/180*pi
		self.B = B/180*pi
		self.C = C/180*pi

		limit1 = np.array([-170,170]);
		limit2 = np.array([-110,110]);
		limit3 = np.array([-220,40]);
		limit4 = np.array([-185,185]);
		limit5 = np.array([-125,125]);
		# limit6 = np.array([-360,360]);
		limit6 = np.array([-360,360])
		self.limit = np.array([limit1,limit2,limit3,limit4,limit5,limit6])
		self.t = np.zeros(6)
		# Inverse 0 3 5 is inverse to SD700E
		# t[0] = -t[0]
		# t[3] = -t[3]
		# t[5] = -t[5]
		self.t = t

		# print(self.t)
	def inverse(self):

		## constants
		d2 = 350;
		d3 = 350;
		# d5 = 74.5;
		d5 = 74.5 + 127.5   # add gripper
		b = 280;
		base = 280;

		# pose matrix
		Rotx = np.array([[1,0,0],[0,cos(self.A),-sin(self.A)],[0,sin(self.A),cos(self.A)]])
		Roty = np.array([[cos(self.B),0,sin(self.B)],[0,1,0],[-sin(self.B),0,cos(self.B)]])
		Rotz = np.array([[cos(self.C),-sin(self.C),0],[sin(self.C),cos(self.C),0],[0,0,1]])
		T1 = np.dot(np.dot(Rotx,Roty),Rotz)
		T1 = np.c_[T1,[self.x,self.y,self.z]]
		# print("T1:")
		# print(T1)



		# pose matrix2 after z+50

		nx=T1[0][0]; ox=T1[0][1]; ax=T1[0][2]; px=T1[0][3];
		ny=T1[1][0]; oy=T1[1][1]; ay=T1[1][2]; py=T1[1][3];
		nz=T1[2][0]; oz=T1[2][1]; az=T1[2][2]; pz=T1[2][3];


		## inverse process (pose matrix2)
		# theta1:2
		theta1_1 = atan2(py-d5*ay, px-d5*ax);
		theta1_2 = atan2(d5*ay-py, d5*ax-px);
		theta1 = np.array([theta1_1,theta1_2]);

		#s3: 2
		#bs3_1:theta1_1  s3_2:theta1_2
		s3_1 = (d2**2 + d3**2 - (-(cos(theta1_1)*ax+sin(theta1_1)*ay)*d5+cos(theta1_1)*px+sin(theta1_1)*py)**2 - (-az*d5+pz-b)**2)/float(2*d2*d3);
		s3_2 = (d2**2 + d3**2 - (-(cos(theta1_2)*ax+sin(theta1_2)*ay)*d5+cos(theta1_2)*px+sin(theta1_2)*py)**2 - (-(-sin(theta1_2)*ax+cos(theta1_2)*ay)*d5+(-sin(theta1_2)*px+cos(theta1_2)*py))**2)/(2*d2*d3);
		# rho: 2
		# rho_1:theta1_1 rho_2:theta1_2
		rho_1 = sqrt((d5*(cos(theta1_1)*ax+sin(theta1_1)*ay)-cos(theta1_1)*px-sin(theta1_1)*py)**2 + (pz-base-d5*az)**2);
		rho_2 = sqrt((d5*(cos(theta1_2)*ax+sin(theta1_2)*ay)-cos(theta1_2)*px-sin(theta1_2)*py)**2 + (pz-base-d5*az)**2);

		# theta2: 4
		# theta2_1,theta2_2: theta1_1,s3_1,rho_1
		# theta2_3,theta2_4: theta1_2,s3_2,rho_2
		theta2_1 = atan2(pz-base-d5*az, d5*(cos(theta1_1)*ax+sin(theta1_1)*ay)-cos(theta1_1)*px-sin(theta1_1)*py) - atan2((d2-s3_1*d3)/float(rho_1),sqrt(1-((d2-s3_1*d3)/float(rho_1))**2));
		theta2_2 = atan2(pz-base-d5*az, d5*(cos(theta1_1)*ax+sin(theta1_1)*ay)-cos(theta1_1)*px-sin(theta1_1)*py) - atan2((d2-s3_1*d3)/float(rho_1),-sqrt(1-((d2-s3_1*d3)/float(rho_1))**2));
		theta2_3 = atan2(pz-base-d5*az, d5*(cos(theta1_2)*ax+sin(theta1_2)*ay)-cos(theta1_2)*px-sin(theta1_2)*py) - atan2((d2-s3_2*d3)/float(rho_2),sqrt(1-((d2-s3_2*d3)/float(rho_2))**2));
		theta2_4 = atan2(pz-base-d5*az, d5*(cos(theta1_2)*ax+sin(theta1_2)*ay)-cos(theta1_2)*px-sin(theta1_2)*py) - atan2((d2-s3_2*d3)/float(rho_2),-sqrt(1-((d2-s3_2*d3)/float(rho_2))**2));

		# c23: 4 s23: 4 theta23 : 4
		# theta23_1, c23_1, s23_1:  theta1_1,theta2_1
		# theta23_2, c23_2, s23_2:  theta1_1,theta2_2
		# theta23_3, c23_3, s23_3:  theta1_2,theta2_3
		# theta23_4, c23_4, s23_4:  theta1_2,theta2_4
		c23_1 = (-(cos(theta1_1)*ax+sin(theta1_1)*ay)*d5 + cos(theta1_1)*px + sin(theta1_1)*py - sin(theta2_1)*d2)/float(d3);
		s23_1 = (cos(theta2_1)*d2 + az*d5 - pz + b)/float(d3);
		c23_2 = (-(cos(theta1_1)*ax+sin(theta1_1)*ay)*d5 + cos(theta1_1)*px + sin(theta1_1)*py - sin(theta2_2)*d2)/float(d3);
		s23_2 = (cos(theta2_2)*d2 + az*d5 - pz + b)/float(d3);
		c23_3 = (-(cos(theta1_2)*ax+sin(theta1_2)*ay)*d5 + cos(theta1_2)*px + sin(theta1_2)*py - sin(theta2_3)*d2)/float(d3);
		s23_3 = (cos(theta2_3)*d2 + az*d5 - pz + b)/float(d3);
		c23_4 = (-(cos(theta1_2)*ax+sin(theta1_2)*ay)*d5 + cos(theta1_2)*px + sin(theta1_2)*py - sin(theta2_4)*d2)/float(d3);
		s23_4 = (cos(theta2_4)*d2 + az*d5 - pz + b)/float(d3);
		theta23_1 = atan2(s23_1, c23_1);
		theta23_2 = atan2(s23_2, c23_2);
		theta23_3 = atan2(s23_3, c23_3);
		theta23_4 = atan2(s23_4, c23_4);

		theta3_1 = theta23_1 - theta2_1;
		theta3_2 = theta23_2 - theta2_2;
		theta3_3 = theta23_3 - theta2_3;
		theta3_4 = theta23_4 - theta2_4;

		# theta4: 8
		# theta4_1,theta4_2: theta1_1 theta2_1 theta23_1
		theta4_1 = atan2(-sin(theta1_1)*ax+cos(theta1_1)*ay, -(s23_1*(cos(theta1_1)*ax+sin(theta1_1)*ay)+c23_1*az));
		theta4_2 = atan2(sin(theta1_1)*ax-cos(theta1_1)*ay, s23_1*(cos(theta1_1)*ax+sin(theta1_1)*ay)+c23_1*az);
		# theta4_3,theta4_4: theta1_1 theta2_2 theta23_2
		theta4_3 = atan2(-sin(theta1_1)*ax+cos(theta1_1)*ay, -(s23_2*(cos(theta1_1)*ax+sin(theta1_1)*ay)+c23_2*az));
		theta4_4 = atan2(sin(theta1_1)*ax-cos(theta1_1)*ay, s23_2*(cos(theta1_1)*ax+sin(theta1_1)*ay)+c23_2*az);
		# theta4_5,theta4_6: theta1_2 theta2_3 theta23_3
		theta4_5 = atan2(-sin(theta1_2)*ax+cos(theta1_2)*ay, -(s23_3*(cos(theta1_2)*ax+sin(theta1_2)*ay)+c23_3*az));
		theta4_6 = atan2(sin(theta1_2)*ax-cos(theta1_2)*ay, s23_3*(cos(theta1_2)*ax+sin(theta1_2)*ay)+c23_3*az);
		# theta4_7,theta4_8: theta1_2 theta2_4 theta23_4
		theta4_7 = atan2(-sin(theta1_2)*ax+cos(theta1_2)*ay, -(s23_4*(cos(theta1_2)*ax+sin(theta1_2)*ay)+c23_4*az));
		theta4_8 = atan2(sin(theta1_2)*ax-cos(theta1_2)*ay, s23_4*(cos(theta1_2)*ax+sin(theta1_2)*ay)+c23_4*az);

		# theta5_1: theta4_1 theta1_1 theta2_1 theta23_1
		s5_1 =  sin(theta4_1)*(- sin(theta1_1)*ax+ cos(theta1_1)*ay) -  cos(theta4_1)*(s23_1*( cos(theta1_1)*ax+ sin(theta1_1)*ay)+c23_1*az);
		c5_1 = c23_1*( cos(theta1_1)*ax+ sin(theta1_1)*ay)-s23_1*az;
		theta5_1 = atan2(s5_1,c5_1);
		# theta5_2: theta4_2 theta1_1 theta2_1 theta23_1
		s5_2 = sin(theta4_2)*(-sin(theta1_1)*ax+cos(theta1_1)*ay) - cos(theta4_2)*(s23_1*(cos(theta1_1)*ax+sin(theta1_1)*ay)+c23_1*az);
		c5_2 = c23_1*(cos(theta1_1)*ax+sin(theta1_1)*ay)-s23_1*az;
		theta5_2 = atan2(s5_2,c5_2);
		# theta5_3: theta4_3 theta1_1 theta2_2 theta23_2
		s5_3 = sin(theta4_3)*(-sin(theta1_1)*ax+cos(theta1_1)*ay) - cos(theta4_3)*(s23_2*(cos(theta1_1)*ax+sin(theta1_1)*ay)+c23_2*az);
		c5_3 = c23_2*(cos(theta1_1)*ax+sin(theta1_1)*ay)-s23_2*az;
		theta5_3 = atan2(s5_3,c5_3);
		# theta5_4: theta4_4 theta1_1 theta2_2 theta23_2
		s5_4 = sin(theta4_4)*(-sin(theta1_1)*ax+cos(theta1_1)*ay) - cos(theta4_4)*(s23_2*(cos(theta1_1)*ax+sin(theta1_1)*ay)+c23_2*az);
		c5_4 = c23_2*(cos(theta1_1)*ax+sin(theta1_1)*ay)-s23_2*az;
		theta5_4 = atan2(s5_4,c5_4);
		# theta5_5: theta4_5 theta1_2 theta2_3 theta23_3
		s5_5 = sin(theta4_5)*(-sin(theta1_2)*ax+cos(theta1_2)*ay) - cos(theta4_5)*(s23_3*(cos(theta1_2)*ax+sin(theta1_2)*ay)+c23_3*az);
		c5_5 = c23_3*(cos(theta1_2)*ax+sin(theta1_2)*ay)-s23_3*az;
		theta5_5 = atan2(s5_5,c5_5);
		# theta5_6: theta4_6 theta1_2 theta2_3 theta23_3
		s5_6 = sin(theta4_6)*(-sin(theta1_2)*ax+cos(theta1_2)*ay) - cos(theta4_6)*(s23_3*(cos(theta1_2)*ax+sin(theta1_2)*ay)+c23_3*az);
		c5_6 = c23_3*(cos(theta1_2)*ax+sin(theta1_2)*ay)-s23_3*az;
		theta5_6 = atan2(s5_6,c5_6);
		# theta5_7: theta4_7 theta1_2 theta2_4 theta23_4
		s5_7 = sin(theta4_7)*(-sin(theta1_2)*ax+cos(theta1_2)*ay) - cos(theta4_7)*(s23_4*(cos(theta1_2)*ax+sin(theta1_2)*ay)+c23_4*az);
		c5_7 = c23_4*(cos(theta1_2)*ax+sin(theta1_2)*ay)-s23_4*az;
		theta5_7 = atan2(s5_7,c5_7);
		# theta5_8: theta4_8 theta1_2 theta2_4 theta23_4
		s5_8 = sin(theta4_8)*(-sin(theta1_2)*ax+cos(theta1_2)*ay) - cos(theta4_8)*(s23_4*(cos(theta1_2)*ax+sin(theta1_2)*ay)+c23_4*az);
		c5_8 = c23_4*(cos(theta1_2)*ax+sin(theta1_2)*ay)-s23_4*az;
		theta5_8 = atan2(s5_8,c5_8);

		# theta6_1: theta4_1 theta1_1 theta2_1 theta23_1
		s6_1 = -sin(theta4_1)*(s23_1*(cos(theta1_1)*nx+sin(theta1_1)*ny)+c23_1*nz)-cos(theta4_1)*(-sin(theta1_1)*nx+cos(theta1_1)*ny);
		c6_1 = -sin(theta4_1)*(s23_1*(cos(theta1_1)*ox+sin(theta1_1)*oy)+c23_1*oz)-cos(theta4_1)*(-sin(theta1_1)*ox+cos(theta1_1)*oy);
		theta6_1 = atan2(s6_1, c6_1);
		# theta6_2: theta4_2 theta1_1 theta2_1 theta23_1
		s6_2 = -sin(theta4_2)*(s23_1*(cos(theta1_1)*nx+sin(theta1_1)*ny)+c23_1*nz)-cos(theta4_2)*(-sin(theta1_1)*nx+cos(theta1_1)*ny);
		c6_2 = -sin(theta4_2)*(s23_1*(cos(theta1_1)*ox+sin(theta1_1)*oy)+c23_1*oz)-cos(theta4_2)*(-sin(theta1_1)*ox+cos(theta1_1)*oy);
		theta6_2 = atan2(s6_2, c6_2);
		# theta6_3: theta4_3 theta1_1 theta2_2 theta23_2
		s6_3 = -sin(theta4_3)*(s23_2*(cos(theta1_1)*nx+sin(theta1_1)*ny)+c23_2*nz)-cos(theta4_3)*(-sin(theta1_1)*nx+cos(theta1_1)*ny);
		c6_3 = -sin(theta4_3)*(s23_2*(cos(theta1_1)*ox+sin(theta1_1)*oy)+c23_2*oz)-cos(theta4_3)*(-sin(theta1_1)*ox+cos(theta1_1)*oy);
		theta6_3 = atan2(s6_3, c6_3);
		# theta6_4: theta4_4 theta1_1 theta2_2 theta23_2
		s6_4 = -sin(theta4_4)*(s23_2*(cos(theta1_1)*nx+sin(theta1_1)*ny)+c23_2*nz)-cos(theta4_4)*(-sin(theta1_1)*nx+cos(theta1_1)*ny);
		c6_4 = -sin(theta4_4)*(s23_2*(cos(theta1_1)*ox+sin(theta1_1)*oy)+c23_2*oz)-cos(theta4_4)*(-sin(theta1_1)*ox+cos(theta1_1)*oy);
		theta6_4 = atan2(s6_4, c6_4);
		# theta6_5: theta4_5 theta1_2 theta2_3 theta23_3
		s6_5 = -sin(theta4_5)*(s23_3*(cos(theta1_2)*nx+sin(theta1_2)*ny)+c23_3*nz)-cos(theta4_5)*(-sin(theta1_2)*nx+cos(theta1_2)*ny);
		c6_5 = -sin(theta4_5)*(s23_3*(cos(theta1_2)*ox+sin(theta1_2)*oy)+c23_3*oz)-cos(theta4_5)*(-sin(theta1_2)*ox+cos(theta1_2)*oy);
		theta6_5 = atan2(s6_5, c6_5);
		# theta6_6: theta4_6 theta1_2 theta2_3 theta23_3
		s6_6 = -sin(theta4_6)*(s23_3*(cos(theta1_2)*nx+sin(theta1_2)*ny)+c23_3*nz)-cos(theta4_6)*(-sin(theta1_2)*nx+cos(theta1_2)*ny);
		c6_6 = -sin(theta4_6)*(s23_3*(cos(theta1_2)*ox+sin(theta1_2)*oy)+c23_3*oz)-cos(theta4_6)*(-sin(theta1_2)*ox+cos(theta1_2)*oy);
		theta6_6 = atan2(s6_6, c6_6);
		# theta6_7: theta4_7 theta1_2 theta2_4 theta23_4
		s6_7 = -sin(theta4_7)*(s23_4*(cos(theta1_2)*nx+sin(theta1_2)*ny)+c23_4*nz)-cos(theta4_7)*(-sin(theta1_2)*nx+cos(theta1_2)*ny);
		c6_7 = -sin(theta4_7)*(s23_4*(cos(theta1_2)*ox+sin(theta1_2)*oy)+c23_4*oz)-cos(theta4_7)*(-sin(theta1_2)*ox+cos(theta1_2)*oy);
		theta6_7 = atan2(s6_7, c6_7);
		# theta6_8: theta4_8 theta1_2 theta2_4 theta23_4
		s6_8 = -sin(theta4_8)*(s23_4*(cos(theta1_2)*nx+sin(theta1_2)*ny)+c23_4*nz)-cos(theta4_8)*(-sin(theta1_2)*nx+cos(theta1_2)*ny);
		c6_8 = -sin(theta4_8)*(s23_4*(cos(theta1_2)*ox+sin(theta1_2)*oy)+c23_4*oz)-cos(theta4_8)*(-sin(theta1_2)*ox+cos(theta1_2)*oy);
		theta6_8 = atan2(s6_8, c6_8);

		# pose matrix from angle 
		#  normally, TT is equal to T1, except py+50

		nxx = cos(theta1_1)*(s23_1*(cos(theta4_1)*c5_1*c6_1-sin(theta4_1)*s6_1)+c23_1*s5_1*c6_1) + sin(theta1_1)*(sin(theta4_1)*c5_1*c6_1+cos(theta4_1)*s6_1);
		oxx = -cos(theta1_1)*(s23_1*(cos(theta4_1)*c5_1*s6_1+sin(theta4_1)*c6_1)+c23_1*s5_1*s6_1) - sin(theta1_1)*(sin(theta4_1)*c5_1*s6_1-cos(theta4_1)*c6_1);
		axx = cos(theta1_1)*(-s23_1*cos(theta4_1)*s5_1+c23_1*c5_1) - sin(theta1_1)*sin(theta4_1)*s5_1;
		pxx = cos(theta1_1)*((-s23_1*cos(theta4_1)*s5_1+c23_1*c5_1)*d5+c23_1*d3+sin(theta2_1)*d2)-sin(theta1_1)*sin(theta4_1)*s5_1*d5;
		nyy = sin(theta1_1)*(s23_1*(cos(theta4_1)*c5_1*c6_1-sin(theta4_1)*s6_1)+c23_1*s5_1*c6_1) - cos(theta1_1)*(sin(theta4_1)*c5_1*c6_1+cos(theta4_1)*s6_1);
		oyy = -sin(theta1_1)*(s23_1*(cos(theta4_1)*c5_1*s6_1+sin(theta4_1)*c6_1)+c23_1*s5_1*s6_1) + cos(theta1_1)*(sin(theta4_1)*c5_1*s6_1-cos(theta4_1)*c6_1);
		ayy = sin(theta1_1)*(-s23_1*cos(theta4_1)*s5_1+c23_1*c5_1) + cos(theta1_1)*sin(theta4_1)*s5_1;
		pyy = sin(theta1_1)*((-s23_1*cos(theta4_1)*s5_1+c23_1*c5_1)*d5+c23_1*d3+sin(theta2_1)*d2) + cos(theta1_1)*sin(theta4_1)*s5_1*d5;
		nzz = c23_1*(cos(theta4_1)*c5_1*c6_1-sin(theta4_1)*s6_1) - s23_1*s5_1*c6_1;
		ozz = -c23_1*(cos(theta4_1)*c5_1*s6_1+sin(theta4_1)*c6_1) + s23_1*s5_1*s6_1;
		azz = -c23_1*cos(theta4_1)*s5_1 - s23_1*c5_1;
		pzz = -(c23_1*cos(theta4_1)*s5_1+s23_1*c5_1)*d5 - s23_1*d3 + cos(theta2_1)*d2 + base;

		TT = np.array([[nxx,oxx,axx,pxx],[nyy,oyy,ayy,pyy],[nzz,ozz,azz,pzz]])
		# print("TT")
		# print(TT)


		# eight different solutions
		# radian
		solution1 = np.array([theta1_1,theta2_1,theta3_1,theta4_1,theta5_1,theta6_1])
		solution2 = np.array([theta1_1,theta2_1,theta3_1,theta4_2,theta5_2,theta6_2])
		solution3 = np.array([theta1_1,theta2_2,theta3_2,theta4_3,theta5_3,theta6_3])
		solution4 = np.array([theta1_1,theta2_2,theta3_2,theta4_4,theta5_4,theta6_4])
		solution5 = np.array([theta1_2,theta2_3,theta3_3,theta4_5,theta5_5,theta6_5])
		solution6 = np.array([theta1_2,theta2_3,theta3_3,theta4_6,theta5_6,theta6_6])
		solution7 = np.array([theta1_2,theta2_4,theta3_4,theta4_7,theta5_7,theta6_7])
		solution8 = np.array([theta1_2,theta2_4,theta3_4,theta4_8,theta5_8,theta6_8])
		solution = np.array([solution1,solution2,solution3,solution4,solution5,solution6,solution7,solution8])
		#print solution


		# angle
		solution = solution/pi*180;
		# Inverse 0 3 5 is inverse to SD700E
		for s in solution:
			s[0] = -s[0]
			s[3] = -s[3]
			s[5] = -s[5]
			# print(s)



		# Screening solutions based on angle constraints
		result_solution = []
		ok = False;
		for s in solution:
			for i in range(len(s)):
				s[i] = round(s[i],2)
				if(s[i]<=self.limit[i][1] and s[i]>=self.limit[i][0]):
					ok = True;
				elif(s[i]<self.limit[i][0] and s[i]+360<=self.limit[i][1] and s[i]+360>=self.limit[i][0]):
					s[i] += 360;
					ok = True;
				elif(s[i]>self.limit[i][1] and s[i]-360>=self.limit[i][0] and s[i]-360<=self.limit[i][1]):
					s[i] -= 360;
					ok = True;
				else:
					ok = False;
					break;
			if(ok):
				# print(s);
				result_solution.append(s)
			# else:
				# print("xx")
		# print(result_solution)

		# choose min L2
		min_iloc = -1
		min_l2 = 0
		for i_r in range(len(result_solution)):
			l2 = 0
			for i in range(6):
				l2 += pow(result_solution[i_r][i]-self.t[i],2)
			if(i_r == 0):
				min_l2 = l2
				min_iloc = i_r
			else:
				if(min_l2 > l2):
					min_l2 = l2
					min_iloc = i_r

		# print(min_iloc)
		return result_solution,min_iloc









inv = Inverse(700,-55,400,-179,28,-2,np.array([0.3,42.8,-49.6,-1.1,68.8,2.2]))
solutions,iloc = inv.inverse()
print(len(solutions))
print(iloc)




# ## jointpose
# # angel
# t1 = 10.58;
# t2 = 47.2;
# t3 = 29.9;
# t4 = -5.85;
# t5 = -73.1;
# t6 = 172.7;
# t1 = -t1; t4 = -t4; t6 = -t6;
# angle = np.array([t1,t2,t3,t4,t5,t6])
# # radian
# t = angle/float(180)*pi;
# print(angle);

# # pose matrix based on jointpose
# s1 = sin(t[0]); c1 = cos(t[0]);
# s2 = sin(t[1]); c2 = cos(t[1]);
# s23 = sin(t[1]+t[2]); c23 = cos(t[1]+t[2]);
# s4 = sin(t[3]); c4 = cos(t[3])
# s5 = sin(t[4]); c5 = cos(t[4])
# s6 = sin(t[5]); c6 = cos(t[5])

# nx1 = c1*(s23*(c4*c5*c6 - s4*s6) + c23*s5*c6) + s1*(s4*c5*c6+c4*s6);
# ny1 = s1*(s23*(c4*c5*c6 - s4*s6) + c23*s5*c6) - c1*(s4*c5*c6+c4*s6);
# nz1 = c23*(c4*c5*c6-s4*s6) - s23*s5*c6;
# ox1 = -c1*(s23*(c4*c5*s6 + s4*c6) + c23*s5*s6) - s1*(s4*c5*s6-c4*c6);
# oy1 = -s1*(s23*(c4*c5*s6 + s4*c6) + c23*s5*s6) + c1*(s4*c5*s6-c4*c6);
# oz1 = -c23*(c4*c5*s6 + s4*c6) + s23*s5*s6;
# ax1 = c1*(-s23*c4*s5 + c23*c5) - s1*s4*s5;
# ay1 = s1*(-s23*c4*s5 + c23*c5) + c1*s4*s5;
# az1 = -c23*c4*s5 - s23*c5;
# px1 = c1*((-s23*c4*s5 + c23*c5)*d5 + c23*d3 + s2*d2) - s1*s4*s5*d5;
# py1 = s1*((-s23*c4*s5 + c23*c5)*d5 + c23*d3 + s2*d2) + c1*s4*s5*d5;
# pz1 = -(c23*c4*s5 + s23*c5)*d5 - s23*d3 + c2*d2 + base;

# T2 = np.array([[nx1,ox1,ax1,px1],[ny1,oy1,ay1,py1],[nz1,oz1,az1,pz1]]);
# print(T2)




