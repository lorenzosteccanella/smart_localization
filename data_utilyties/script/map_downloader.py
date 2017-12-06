import overpass 
api = overpass.API()
map_query = overpass.MapQuery(50.746,7.154,50.748,7.157)
way_query = overpass.WayQuery()
response = api.Get(way_query)

print(response)
