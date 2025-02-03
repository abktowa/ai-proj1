#### BOSTON DATA ####

boston_stations.csv contains every station on the MBTA Subway, not including the Silver Line. Latitude and Longitude data comes from Wikipedia articles for each respective station, rounded to 4 decimal places. Degree-minutes-seconds data was converted to decimal. Stations included are all of those open as of July 31, 2023.

boston_links.csv contains every track connection between the stations in boston_stations.csv. Station IDs correspond to the ID numbers in the station CSV file. Distances of each track connections are given in miles and kilometers. Mileage comes from the MBTA Blue Book (https://old.mbta.com/uploadedfiles/documents/2014%20BLUEBOOK%2014th%20Edition(1).pdf), except those below which were hand-measured using Google Maps tool for measuring distance:

	* Orange Line between Wellington and Assembly; and between Assembly and Sullivan Square (data source pre-dates opening of Assembly station)
	* Many Green Line distances (data source gives distances skipping over several stations; intermediate distances were hand-measured; new branches between Lechmere and Union Sq, and Lechmere and Medford pre-date data source)

Because of the different data sources, straight line distance (SLD) (calculated using latitude and longitude) is longer than the MBTA-reported or hand-measured distance in some links.

#### LONDON DATA ####

london_stations.csv contains every station on the London Underground (not DLR, Crossrail, TfL Rail, or Overground). Latitude and Longitude data comes from Wikipedia articles for each respective station, rounded to 4 decimal places. Degree-minutes-seconds data was converted to decimal.

london_links.csv contains every track connection between the stations in london_stations.csv. Station IDs correspond to the ID numbers in the station CSV file. Distances of each track connections are given in miles and kilometers. Kilometer distances come from the Inter Station Database file found here (https://www.whatdotheyknow.com/request/distance_between_adjacent_underg) or from the Working Timetables found on the Transport For London website (https://tfl.gov.uk/corporate/publications-and-reports/working-timetables). Also helpful was this site: https://www.whatdotheyknow.com/request/distance_between_tfl_stations

An additional link was created between Bank and Monument stations to represent the free transfer available. The link has a "length" of 10 meters.

Because of the different data sources, straight line distance (SLD) (calculated using latitude and longitude) is longer than the TfL-reported distances in some links. The difference only exceeds 100 meters in 36 links, and exceeds 200 meters only in 5 links.

