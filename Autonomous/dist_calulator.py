
from math import radians, cos, sin, asin, sqrt
def distance(arr1, arr2):
	


	lon1 = radians(arr1[1])
	lon2 = radians(arr2[1])
	lat1 = radians(arr1[0])
	lat2 = radians(arr2[0])
	
	# Haversine formula 
	dlon = lon2 - lon1 
	dlat = lat2 - lat1
	a = sin(dlat / 2)**2 + cos(lat1) * cos(lat2) * sin(dlon / 2)**2

	c = 2 * asin(sqrt(a)) 
	
	# Radius of earth in kilometers. Use 3956 for miles
	r = 6371
	
	# calculate the result
	return(c * r)*1000
	
	
# driver code 


#print(distance([53.32055555555556, -1.7297222222222221], [53.31861111111111, -1.6997222222222223]))

