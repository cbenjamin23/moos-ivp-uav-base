
# Extract Telemetry Data from alog file to csv file 

This will extract to the format: Time,Var, Value, and can be done calling the script `alog_extract_telemetry_csv.sh` under scripts.


Use python script  `valuePlotter.py` to generate plots

NOT WORKING:
Use the alias:
```bash
VARS="NAV_* DESIRED_*"
alias extract_alog_nav_csv="aloggrep *.alog $VARS --tvv --csc > telemetry_data.csv"
```