import pandas as pd
import re

'''
Data format:
------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
Project: 
Program: 
Profile: 
Source:   
SolFile:  
ProcessInfo: 

Datum:       
Master 1:   
Remote:    
IMU to GNSS Antenna Lever Arms:
Body to IMU Rotations:
UTC Offset:

Map projection Info:
  UTM Zone:    

  VX-ECEF   VY-ECEF   VZ-ECEF   X-ECEF-Vec   Y-ECEF-Vec   Z-ECEF-Vec        Azimuth           ECTX           ECTY           ECTZ          Pitch        PtchSep           Roll        RollSep         Latitude        Longitude       X-ECEF       Y-ECEF       Z-ECEF        H-Ell       UTCTime      Easting     Northing        Az(1->2)        Az(2->1)
    (m/s)     (m/s)     (m/s)          (m)          (m)          (m)          (deg)          (deg)          (deg)          (deg)          (deg)          (deg)          (deg)          (deg)       (+/-D M S)       (+/-D M S)          (m)          (m)          (m)          (m)         (sec)          (m)          (m)         (D-M-S)         (D-M-S)
'''

class SurveyData:
    def __init__(self, path):
        self.path = path
        self.dataframe = None
        self.column_headers = None
        self._load_data()

    def _load_data(self):
        with open(self.path, 'r') as file:
            lines = file.readlines()

        data_start = False
        data_lines = []
        for line in lines:
            if re.match(r'\s*VX-ECEF', line):
                data_start = True
                self.column_headers = re.split(r'\s{2,}', line.strip())
                continue
            if data_start:
                if re.match(r'\s*[-0-9]', line):
                    data_lines.append(re.split(r'\s{2,}', line.strip()))
        self.dataframe = pd.DataFrame(data_lines, columns=self.column_headers)

    def get_dataframe(self):
        return self.dataframe

if __name__ == '__main__':
    path_to_file = 'path'
    survey_data = SurveyData(path_to_file)
    df = survey_data.get_dataframe()
    print(df)
