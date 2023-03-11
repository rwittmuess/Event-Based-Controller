function result = control1()
% control vertical hopper

global h_axes body leg
global dt time x y xd yd 
global hip_torque leg_angle body_angle leg_angled body_angled
global leg_state foot_x foot_y leg_lengthd leg_length rest_leg_length
global control_state height_desired leg_angle_desired last_bounce_time
global last_touchdown_time last_takeoff_time max_height last_max_height
global speed_desired
global p_error i_error d_error

% control_state values
init = 0;
in_air = 1;
on_ground_going_down = 2;
on_ground_going_up = 3;

hip_air_k = 1000;
hip_air_b = 25;
hip_grnd_k = 100;
hip_grnd_b = 10;

leg_length_default = 0.5;

leg_length_gain = 0.0;

rest_leg_length = leg_length_default;
hip_torque = 0;   

foot_y_new = y - rest_leg_length*cos( leg_angle );
leg_length_new = sqrt( (x - foot_x)^2 + (y - foot_y)^2 );

% initialization
if control_state == init
  control_state = in_air;
  result = control_state;
  return;
end;

if control_state == in_air
  if foot_y_new < 0
    last_touchdown_time = time;
    if yd <= 0
      control_state = on_ground_going_down;
    else
      control_state = on_ground_going_up;
    end;
    result = control_state;
    return;
  end;
  %leg_angle_desired = -0.5;
  gain=0.022;
  p_error   = [p_error; speed_desired-xd];
  i_error   = [i_error; sum(p_error)];
  if isempty(d_error); d_error=0; else d_error=[d_error; p_error(end)-p_error(end-1)]; end
  leg_angle_desired = asin((xd*last_bounce_time/2 + gain*(xd-speed_desired))/(rest_leg_length))...
                        -0.00006*i_error(end)...
                        +10*d_error(end);

  hip_torque = hip_air_k*(leg_angle - leg_angle_desired) + ...
                             hip_air_b*leg_angled;
  if ( y > max_height )
    max_height = y;
  end;
  if ( yd < 0 )
    last_max_height = max_height;
  end;
end;

if control_state == on_ground_going_down
  if leg_length_new > rest_leg_length
    control_state = in_air;
    max_height = y;
    result = control_state;
    last_takeoff_time = time;
    return;
  end;
  if yd > 0
    control_state = on_ground_going_up;
    result = control_state;
    return;
  end;
  hip_torque = hip_grnd_k*(-body_angle) - hip_grnd_b*body_angled;
end;

if control_state == on_ground_going_up
  rest_leg_length =  sqrt((1*9.81*height_desired-1*9.81*y-1/2*1*yd.^2)*2/200) + leg_length;
  if leg_length_new > rest_leg_length
    control_state = in_air;
    max_height = y;
    result = control_state;
    last_takeoff_time = time;
    if ( last_touchdown_time > 0 )
      last_bounce_time = last_takeoff_time - last_touchdown_time;
    end;
    return;
  end;
  if yd < 0
    control_state = on_ground_going_down;
    result = control_state;
    return;
  end;
  hip_torque = hip_grnd_k*(-body_angle) - hip_grnd_b*body_angled;
end;
