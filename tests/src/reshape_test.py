import numpy as np


a = np.array([[ [1,2,3,4],[3,4,5,6],[0,0,0,0] ],
	[[1,2,3,4],[3,4,5,6],[1,1,1,1]],
	[[1,2,3,4],[3,4,5,6],[2,2,2,2]] ])
print(a.shape)
print(a)
s = a.shape

b=np.reshape(a.flatten(),s)
print("reshape")
print(b.shape)
print(b)
