import math
import csv

class Station:
	def __init__(self, id, name, lat, long):
		self.id = id
		self.name = name
		self.lat = lat
		self.long = long
	
	def __hash__(self):
		return hash(id(self))
		
	def __str__(self):
		return self.name + '(' + str(self.id) + ')'
	
	def __repr__(self):
		return self.name + '(' + str(self.id) + ')'

class Link:
	def __init__(self, u, v, weight, line):
		self.start = u
		self.end = v
		self.distance = weight
		self.line = line
		
	def __str__(self):
		return str(self.start) + '<-->' + str(self.end) + '(' + self.line + ')'
	
	def __repr__(self):
		return str(self.start) + '<-->' + str(self.end) + '(' + self.line + ')'
		
	def endpoints(self):
		return (self.start, self.end)
		
	def opposite(self, v):
		if v not in self.endpoints():
			raise ValueError('Argument is neither endpoint')
		result = self.start if v == self.end else self.end
		return result
	
	def get_distance(self):
		return self.distance
		
class SubwayMap:
	def __init__(self):
		self.stations = {} # Maps ID to Station object
		self.links = {} # Maps Station to List of (Station, Link) tuples
		self.link_list = [] # List of links, stored separately from stations
	
	def num_stations(self):
		"""Return the number of stations on the map"""
		return len(self.stations)
	
	def get_stations(self):
		"""Return a view containing all of the Stations on the map"""
		return self.stations.values()
		
	def get_station_by_id(self, num):
		"""Return the station with the given ID (None if no station has that ID)."""
		return self.stations.get(num)
		
	def get_station_by_name(self, name):
		"""Return the station with the given name (None if no station has that ID)."""
		for station in self.get_stations():
			if station.name == name:
				return station
		return None
		
	def num_links(self):
		"""Returns the number of links on the SubwayMap"""
		total = 0
		for inner_list in self.links.values():
			total += len(inner_list)
		return total // 2
		
	def get_links(self):
		"""A generator that yields every Link in the SubwayMap"""
		for link in self.link_list:
			yield link
		
	def get_links_between(self, u, v):
		"""A generator that yields all Links between the two Stations given as parameters.
		Note that if there are multiple links between the two stations, then each link will be provided.
		"""
		adj = self.links[u]
		for pair in adj:
			if pair[0] == v:
				yield pair[1]
		
	def degree(self, v):
		"""Returns the degree for the given Station (the number of Links associated with that Station)"""
		return len(self.links[v])
		
	def incident_links(self, v):
		"""A generator that yields each Link associated with the given Station"""
		adj = self.links[v]
		for pair in adj:
			yield pair[1]
		
	def adjacent_stations(self, v):
		"""A generator that yields each Station that is adjacent (directly connected by a Link) to the given Station. 
		Note that if a Station is adjacent to the given Station by more than one link, then it will be yielded once per Link.
		"""
		adj = self.links[v]
		for pair in adj:
			yield pair[0]
		
	def insert_station(self, id, name, lat, long):
		"""DO NOT USE -- Used by the build_map() functions to create the maps"""
		vertex = Station(id, name, lat, long)
		self.stations[id] = vertex
		self.links[vertex] = []
		
	def insert_link(self, u, v, weight, line):
		"""DO NOT USE -- Used by the build_map() functions to create the maps"""
		link = Link(u, v, weight, line)
		self.links[u].append( (v, link) )
		self.links[v].append( (u, link) )
		
def straight_line_distance(station1, station2):
	"""Calculate the straight line distance (in km) between two subway stations.
	
	Uses the computational formula from Wikipedia (link below) due to
	the high likelihood that two stations are close together. 
	https://en.wikipedia.org/wiki/Great-circle_distance#Computational_formulas
	"""
	EARTH_RADIUS = 6371 # kilometers
	
	s1_lat = math.radians(station1.lat)
	s1_long = math.radians(station1.long)
	s2_lat = math.radians(station2.lat)
	s2_long = math.radians(station2.long)
	
	delta_lat = abs(s1_lat - s2_lat)
	delta_long = abs(s1_long - s2_long)
	
	under_root = (math.sin(delta_lat/2)**2) + (math.cos(s1_lat) * math.cos(s2_lat) * math.sin(delta_long/2)**2)
	central_angle = 2 * math.asin(math.sqrt(under_root))
	
	distance = EARTH_RADIUS * central_angle
	
	return distance

def build_boston_map():
	"""Returns a SubwayMap object representing the Boston T map"""
	boston_station_file = "../data/boston_stations.csv"
	boston_link_file = "../data/boston_links.csv"
	
	return build_map(boston_station_file, boston_link_file)
	
def build_london_map():
	"""Returns a SubwayMap object representing the London Underground map"""
	london_station_file = "../data/london_stations.csv"
	london_link_file = "../data/london_links.csv"
	
	return build_map(london_station_file, london_link_file)
	
def build_map(station_file, link_file):
	"""DO NOT USE -- This method is called by the build_boston_map() and build_london_map() functions."""
	new_map = SubwayMap()
	
	with open(station_file, newline='') as file:
		stations = csv.reader(file)
		
		next(stations) # Skip header row
		
		for row in stations:
			idnum = int(row[0])
			name = row[1]
			latitude = float(row[2])
			longitude = float(row[3])
			
			new_map.insert_station(idnum, name, latitude, longitude)
		
	with open(link_file, newline='') as file:
		links = csv.reader(file)
		
		next(links) # Skip header row
		
		for row in links:
			deprecated_row = False
			name1 = row[0]
			id1 = int(row[1])
			name2 = row[2]
			id2 = int(row[3])
			
			route = row[4]
			
			distance_miles = float(row[5])
			distance_km = float(row[6])

			if len(row) > 7:
				if row[7] == "DEP":
					deprecated_row = True
			
			if not deprecated_row:
				station1 = new_map.get_station_by_id(id1)
				station2 = new_map.get_station_by_id(id2)
				
				new_map.insert_link(station1, station2, distance_km, route)
	
	return new_map
