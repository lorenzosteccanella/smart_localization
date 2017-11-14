  

d1=10
d2=20
d3=6

landmarks  = [[10.0, 0.0], [20.0, +20.0], [+1.0, +1.0]]


def Trilateration_2D(d1,d2,d3):

	i1=landmarks[0][0]
	j1=landmarks[0][1]
	i2=landmarks[1][0]
	j2=landmarks[1][1]
	i3=landmarks[2][0]
	j3=landmarks[2][1]

	A=((d1**2-d2**2) + (i2**2-i1**2) + (j2**2-j1**2)) * (2*j3-2*j2)
	B=((d2**2-d3**2) + (i3**2-i2**2) + (j3**2-j2**2) ) *(2*j2-2*j1)
	C1=(2*i2-2*i3)
	C2=(2*j2-2*j1)
	C3=(2*i1-2*i2)
	C4=(2*j3-2*j2)
	print(C3,C4)
	C=((2*i2-2*i3)(2*j2-2*j1))-((2*i1-2*i2)(2*j3-2*j2))

	x =( A - B ) / C
	y = ((d1**2-d2**2) + (i2**2-i1**2) + (j2**2-j1**2) + x*(2*i1-2*i2)) / (2*j2-2*j1)

	return x,y


print(Trilateration_2D(d1,d2,d3));

