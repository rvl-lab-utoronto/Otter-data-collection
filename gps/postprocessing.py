import pandas as pd
from gmplot import gmplot

ascii_file_path = r"PATH"
gga_file_path = r"PATH"

# read and parse the ASCII GNSS data
def read_gnss_data(file_path):
    coordinates = []
    with open(file_path, 'r') as file:
        for line in file:
            if line.startswith('#BESTPOSA'):
                parts = line.split(',')
                if len(parts) > 9:
                    try:
                        lat = float(parts[11])
                        lon = float(parts[12])
                        lat_stddev = float(parts[17])
                        lon_stddev = float(parts[18])
                        coordinates.append((lat, lon, lat_stddev, lon_stddev))
                    except (ValueError, IndexError):
                        continue
    return coordinates

# read and parse the GGA file
def read_gga_file(file_path):
    coordinates = []
    with open(file_path, 'r') as file:
        for line in file:
            if line.startswith('$GPGGA'):
                parts = line.split(',')
                if len(parts) > 5:
                    try:
                        lat_deg = float(parts[2][:2])
                        lat_min = float(parts[2][2:])
                        lat = lat_deg + lat_min / 60.0
                        if parts[3] == 'S':
                            lat = -lat

                        lon_deg = float(parts[4][:3])
                        lon_min = float(parts[4][3:])
                        lon = lon_deg + lon_min / 60.0
                        if parts[5] == 'W':
                            lon = -lon

                        coordinates.append((lat, lon))
                    except (ValueError, IndexError):
                        continue
    return coordinates

gnss_data = read_gnss_data(ascii_file_path)
gga_data = read_gga_file(gga_file_path)

latitudes1 = [coord[0] for coord in gnss_data]
longitudes1 = [coord[1] for coord in gnss_data]

latitudes2 = [coord[0] for coord in gga_data]
longitudes2 = [coord[1] for coord in gga_data]

api_key = 'KEY'

if latitudes1 and longitudes1:
    gmap = gmplot.GoogleMapPlotter(latitudes1[0], longitudes1[0], 10, apikey=api_key)

    # Plot the raw trajectory on the map with red dots
    gmap.scatter(latitudes1, longitudes1, '#FF0000', size=5, marker=False)

    # Plot the postprocessed trajectory on the map with green dots
    gmap.scatter(latitudes2, longitudes2, '#00FF00', size=5, marker=False)

    gmap.draw(r"PATH")
else:
    print("No valid coordinates found to plot.")
