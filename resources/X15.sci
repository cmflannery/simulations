clear

funcprot(0)

//
// standard atmosphere model (SI units)
//
function [density,temperature,sos] = STDATM(altitude)
    
    R_air = 287;            // gas constant [J/kg/K]
    gamma_air = 1.4;        // ratio of specific heats
    g0 = 9.8;               // gravity constant [m/s]
    
    layer = -1.0;           // gradient layer
    gradient = -0.0065;
    altitude_base = 0.0;
    temperature_base = 288.16;
    density_base = 1.2250;
    
    if (altitude > 11000.0) then
        layer = 1.0;       // isothermal layer
        altitude_base = 11000.0;
        temperature_base = 216.66;
        density_base = 0.3648;
    end
    
    if (altitude > 25000.0) then
        layer = -1.0;      // gradient layer
        gradient = 0.003;
        altitude_base = 25000.0;
        temperature_base = 216.66;
        density_base = 0.04064;
    end
    
    if (altitude > 47000.0) then
        layer = 1.0;       // isothermal layer
        altitude_base = 47000.0;
        temperature_base = 282.66;
        density_base = 0.001476;
    end
    
    if (altitude > 53000.0) then
        layer = -1.0;      // gradient layer
        gradient = -0.0045;
        altitude_base = 53000.0;
        temperature_base = 282.66;
        density_base = 0.0007579;
    end
    
    if (altitude > 79000.0) then
        layer = 1.0;       // isothermal layer
        altitude_base = 79000.0;
        temperature_base = 165.66;
        density_base = 0.0000224;
    end
    
    if (altitude > 90000.0) then
        layer = -1.0;      // gradient layer
        gradient = 0.004;
        altitude_base = 90000.0;
        temperature_base = 165.66;
        density_base = 0.00000232;
    end
    
    if (layer < 0.0) then
        temperature = temperature_base + gradient*(altitude - altitude_base);
        pow = -1.0*(g0/gradient/R_air + 1.0);
        density = density_base*(temperature/temperature_base)^pow;
    else
        temperature = temperature_base;
        pow = -1.0*g0*(altitude - altitude_base)/R_air/temperature;
        density = density_base*exp(pow);
    end
    sos = sqrt(gamma_air*R_air*temperature);
    
endfunction

//
// 3DOF Equations of Motion
//
function dxdt = EOM(x)
    
    g = 9.8;            // sea-level gravity [m/s/s]
    Re = 6378000.0;     // Earth radius [m]
    Isp = 260.0;        // specific impulse [s] 
    Sref = 18.6;        // wing reference area [m*m] 

    time = x(1);        // time [sec]
    
    V = x(2);           // airspeed [m/s]
    gam = x(3);         // vertical flight path angle [rad]
    chi = x(4);         // flight path heading [rad]
    
    lat = x(5);         // latitude [rad]
    lon = x(6);         // longitude [rad]
    alt = x(7);         // altitude [m]
    
    mass = x(8);        // mass [kg]
    heat = x(9);        // heat [J]
    
    Thrust = x(10);     // thrust [N]
    epsilon = x(11);    // thrust angle [rad]
    CL = x(12);         // lift coefficient
    mu = x(13);         // bank angle [rad]
    
    R = alt + Re;       // radius from Earth center [m]
    
    dxdt = zeros(13,1);
    
    // atmosphere model
    
    [dens,tempr,sos] = STDATM(alt);
    qbar = 0.5*dens*V*V;    // dynamic pressure [N/m2]
    
    // aerodynamics model
    
    M = V/sos;
    CD0 = 0.04 + 0.2*M*M*exp(-M*M);
    K = 0.2 + 0.13*M;
    CD = CD0 + K*CL*CL; 
    Cf = CD/10;
    Drag = qbar*Sref*CD;    // drag force
    Lift = qbar*Sref*CL;    // lift force
        
    // equations of motion
    
    dxdt(1) = 1.0;
    
    dxdt(2) = (Thrust*cos(epsilon) - Drag)/mass - g*(Re/R)*(Re/R)*sin(gam);
    dxdt(3) = (Lift + Thrust*sin(epsilon))*cos(mu)/(V*mass) + (V/R - (g/V)*(Re/R)*(Re/R))*cos(gam);
    dxdt(4) = (Lift + Thrust*sin(epsilon))*sin(mu)/(V*mass*cos(gam)) + (V/R)*cos(gam)*sin(chi)*tan(lat);
    
    dxdt(5) = (V/R)*cos(gam)*cos(chi);
    dxdt(6) = (V/R)*cos(gam)*sin(chi)/cos(lat);
    dxdt(7) = V*sin(gam);
    
    dxdt(8) = -1.0*abs(Thrust)/g/Isp;
    dxdt(9) = 0.5*qbar*V*Sref*Cf;
    
    dxdt(10) = 0.0;
    dxdt(11) = 0.0;
    dxdt(12) = 0.0;
    dxdt(13) = 0.0;
    
endfunction

//
// main program
//

// initial conditions at launch

x = zeros(13,1);    // state vector

x(1) = 0.0;                 // time [sec]
x(2) = 230.0;               // drop airspeed [m/s]
x(3) = 0.0*%pi/180.0;       // vertical flight path angle [rad]

x(4) = -170.0*%pi/180.0;    // flight path heading [rad]
x(5) = 39.33*%pi/180.0;     // latitude [rad]
x(6) = -117.48*%pi/180.0;   // longitude [rad]

x(7) = 13700;               // drop altitude [m]
    
x(8) = 15000.0;             // mass [kg]
x(9) = 0.0;                 // heat [J]
    
x(10) = 262000.0;           // thrust [N]
x(11) = 0.0;                // thrust angle [rad]
x(12) = 0.3;                // lift coefficient
x(13) = 0.0;                // bank angle [rad]

dt = 0.5;                   // time step [sec]
Re = 6378000.0;             // Earth radius [m]

flight_phase = 0;           // mission phase

for i = 1:900
    
    // save state vector for plotting
    
    xs(:,i) = x;
    
    // find other variables of interest
    
    dx = EOM(x);
    
    Rd = Re + x(7);              // radius [m]  
    at(i) = dx(2)/9.8 + (Re/Rd)*(Re/Rd)*sin(x(3));  // tangential accel [g's]
    an(i) = x(2)*dx(3)/9.8 - ((x(2)*x(2)/(9.8*Rd) - (Re/Rd)*(Re/Rd)))*cos(x(3));  // normal accel [g's]
    ac(i) = x(2)*cos(x(3))*dx(4)/9.8;       // lateral accel [g's]
    ac(i) = ac(i) - x(2)*x(2)*cos(x(3))*cos(x(3))*sin(x(4))*tan(x(5))/(9.8*Rd);
    
    heat_rate(i) = dx(9);       // heating rate [W]
    climb_rate(i) = dx(7);      // climb rate [m/s]
    
    [dens,tempr,sos] = STDATM(x(7));
    
    qbar(i) = 0.5*dens*x(2)*x(2);       // dyn. pressure [N/m/m]
    Mach(i) = x(2)/sos;                // Mach number
    stag_temp(i) = tempr*(1.0 + 0.2*Mach(i)*Mach(i)); // stagnation temp. [K]
    
    // set control variables
    
    select flight_phase
        
    case 0      // launch + 2g pitch up
        
        x(12) = 2.0*x(8)*9.8/(qbar(i)*18.6);
        if (x(12) > 0.8) then
            x(12) = 0.8;
        end

        if (x(3) > 34.0*%pi/180.0) then
            flight_phase = 1;
        end
        
    case 1      // hold flight path angle until engine out
        
        x(12) = x(8)*9.8*cos(34.0*%pi/180)/(qbar(i)*18.6);
        if (x(12) > 0.8) then
            x(12) = 0.8;
        end
        
        if (x(1) > 85.0) then
            x(12) = 0.0;
            x(10) = 0.0;                // shut off engine
            flight_phase = 2;
        end
        
    case 2      // roll left
        
        if (x(1) > 240.0) then               
            x(13) = -30*%pi/180;        // bank 30 deg
            flight_phase = 3;
        end
        
    case 3      // recover
        
        x(12) = 5.0*x(8)*9.8/(qbar(i)*18.6);    // 5g turn
        if (x(12) > 0.8) then
            x(12) = 0.8;
        end
        
        if (climb_rate(i) > -200.0) then      
            x(13) = 0.0;                // return wings level
            flight_phase = 4;
        end
        
    case 4
        x(12) = x(8)*9.8/(qbar(i)*18.6);
        if (x(12) > 0.8) then
            x(12) = 0.8;
        end
         
    end
  
    // next step
    
    r1 = dt*EOM(x);
    r2 = dt*EOM(x + 0.5*r1);
    r3 = dt*EOM(x + 0.5*r2);
    r4 = dt*EOM(x + r3);
    x = x + (r1 + 2.0*r2 + 2.0*r3 + r4)/6.0;
    
end

