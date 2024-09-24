# GPS postprocessing piepline 

Raw bag file -> Novatel Application Suite Converter -> Novatel Inertial Explorer Post Processing

For the first step, to save loading time, extract only the required ros topic: `/novatel/oem7/oem7raw`

Create a yaml file such as the following `output.yaml`

```
output_bags:
- uri: /home/aj/output/aug_16_utm  # desired output dir
  topics: [/novatel/oem7/oem7raw] # this is the topic needed for Novatel GPS conversion
```

Use the command `ros2 bag convert -i <input bag dir>' -o output.yaml`

After you obtain the filtered bag, use Novatel Convert Software by selecting OEM7, and input the filtered bag. After loading successfully, select all conversion formats and click convert.

The converted files (binary, ascii, rinex, etc.) will be saved in the directory you specify in the Convert software. Then input the binary file into the Novatel Inertial Explorer Software (Note this requires a license) and follow the New Project Wizard instructions. 


