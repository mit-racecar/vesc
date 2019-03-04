# Vedder VESC open source motor controller state (telemetry)

# fault codes
int32 FAULT_CODE_NONE=0
int32 FAULT_CODE_OVER_VOLTAGE=1
int32 FAULT_CODE_UNDER_VOLTAGE=2
int32 FAULT_CODE_DRV=3
int32 FAULT_CODE_ABS_OVER_CURRENT=4
int32 FAULT_CODE_OVER_TEMP_FET=5
int32 FAULT_CODE_OVER_TEMP_MOTOR=6

# plase follow the bledc firwmare: commands.c
float32 temp_fet             
float32 temp_motor           
float32 avg_motor_current    
float32 avg_input_current    
float32 avg_id               
float32 avg_iq               
float32 duty_cycle_now       
float32 speed #rpm                  
float32 INPUT_VOLTAGE         
float32 amp_hours         
float32 amp_hours_charged         
float32 watt_hours
float32 watt_hours_charged
float32 tachometer
float32 tachometer_abs
int32   fault_code
float32 pid_pos_now
int8    controller_id
float32 NTC_TEMP_MOS1
float32 NTC_TEMP_MOS2
float32 NTC_TEMP_MOS3