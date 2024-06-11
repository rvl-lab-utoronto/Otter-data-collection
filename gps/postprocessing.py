import pandas as pd
import gmplot
import math
from parser import SurveyData

def dms_to_decimal(dms_str):
    parts = dms_str.split()
    degrees = float(parts[0])
    minutes = float(parts[1])
    seconds = float(parts[2])
    decimal = degrees + (minutes / 60) + (seconds / 3600)
    return decimal

def calculate_end_point(lat, lon, heading, distance=0.001):
    """ Calculate the end point given a starting point, heading, and distance """
    R = 6371e3  # Radius of the Earth in meters
    distance = distance * 1000  # Convert distance to meters

    lat = math.radians(lat)
    lon = math.radians(lon)
    heading = math.radians(heading)

    end_lat = math.asin(math.sin(lat) * math.cos(distance / R) + math.cos(lat) * math.sin(distance / R) * math.cos(heading))
    end_lon = lon + math.atan2(math.sin(heading) * math.sin(distance / R) * math.cos(lat), math.cos(distance / R) - math.sin(lat) * math.sin(end_lat))

    end_lat = math.degrees(end_lat)
    end_lon = math.degrees(end_lon)

    return end_lat, end_lon

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

def read_heading_data(file_path):
    headings = []
    with open(file_path, 'r') as file:
        for line in file:
            if line.startswith("#HEADING2A"):
                parts = line.split(',')
                if len(parts) > 16:
                    try:
                        heading = float(parts[16])
                        headings.append(heading)
                    except (ValueError, IndexError):
                        continue
    return headings

def plot_gnss_data(parsed_file_path, ascii_file_path, heading_file_path, api_key, output_path):
    # Load the data from the text file using the SurveyData class
    survey_data = SurveyData(parsed_file_path)
    df = survey_data.get_dataframe()

    # Extract latitude, longitude, and heading from the dataframe
    latitudes = []
    longitudes = []
    headings = []
    for index, row in df.iterrows():
        try:
            lat_dms = row['Latitude'].strip()
            lon_dms = row['Longitude'].strip()
            lat = dms_to_decimal(lat_dms)
            lon = dms_to_decimal(lon_dms)
            heading = float(row['Azimuth'])  # Assuming Azimuth column represents heading
            latitudes.append(lat)
            longitudes.append(lon)
            headings.append(heading)
        except KeyError:
            continue

    # Create the gmplot object
    gmap = gmplot.GoogleMapPlotter(latitudes[0], longitudes[0], 13, apikey=api_key)

    # Plot points from the ASCII file
    ascii_coordinates = read_gnss_data(ascii_file_path)
    ascii_latitudes = [coord[0] for coord in ascii_coordinates]
    ascii_longitudes = [coord[1] for coord in ascii_coordinates]

    # Plot the points from the ASCII file in red
    gmap.scatter(ascii_latitudes, ascii_longitudes, color='red', size=40, marker=False)

    # Plot points and headings from the parsed data frame in green
    gmap.scatter(latitudes, longitudes, color='green', size=40, marker=False)
    for lat, lon, heading in zip(latitudes, longitudes, headings):
        end_lat, end_lon = calculate_end_point(lat, lon, heading)
        gmap.plot([lat, end_lat], [lon, end_lon], color='green')

    # Extract heading information from the heading ASCII file
    headings_from_file = read_heading_data(heading_file_path)
    for lat, lon, heading in zip(ascii_latitudes, ascii_longitudes, headings_from_file):
        end_lat, end_lon = calculate_end_point(lat, lon, heading)
        gmap.plot([lat, end_lat], [lon, end_lon], color='red')

    # Draw the map to an HTML file
    gmap.draw(output_path)

