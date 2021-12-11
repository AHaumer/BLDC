within BLDC.Utilities;
block ElectronicCommutator "Polyphase electronic commutator"
  extends BLDC.BaseBlocks.BaseElectronicCommutator;
  import Modelica.Units.SI;
  import Modelica.Constants.pi;
  import Modelica.Constants.eps;
  import Modelica.ComplexMath.j;
  import Modelica.ComplexMath.arg;
  import Modelica.Math.wrapAngle;
protected
  parameter SI.Angle wrappedOrientation[m]=wrapAngle(orientation, true) "Orientation of phases in the range [0,2*pi)";
  SI.Angle rotorPosition=wrapAngle(arg(Modelica.ComplexMath.sum({if uC[k] then Modelica.ComplexMath.exp(j*orientation[k]) else Complex(0) for k in 1:m})), true)
    "Rotor position in the range [0, 2*pi]";
  SI.Angle diff[m] "Difference between orientation and rotor position";
algorithm
  diff:=wrapAngle(wrappedOrientation - fill(rotorPosition, m), false);
  diff:={if abs(abs(diff[k]) - pi) < eps then 0 else diff[k] for k in 1:m};
equation
  fire_p={if diff[k]>eps then internalPWM elseif diff[k]<-eps then not internalPWM else false for k in 1:m};
  fire_n={if diff[k]>eps then not internalPWM elseif diff[k]<-eps then internalPWM else false for k in 1:m};
  annotation (Documentation(info="<html>
<p>
For every Hall signal <code>uC[k]=true</code>, the phasor <code>exp(-j*orientation[k])</code> is added, else zero. 
Taking the argument (angle) of the resulting phasor, the rotor position with an uncertainty of <code>&#177;&pi;/(2*m)</code> is determined.
</p>
<p>
The fire signals are determined by comparing the rotor position is compared with the orientation of phases. 
Phase(s) aligned with the rotor position (positive or negative) are set inactive (open). 
Phases ahead of the rotor position are connected with <code>pwm</code>,
phases behind of the rotor position are connected with <code>not pwm</code>. 
</p>
<p>
The signal <code>pwm</code> is either determined by the input, or set to the parameter <code>ConstantPWM</code> (determining the direction).
</p>
</html>"), Icon(graphics={
        Line(points={{-60,30},{-60,50},{0,50},{0,30},{60,30},{60,50}}, color={255,
              0,255}),
        Line(points={{-60,-10},{-20,-10},{-20,10},{40,10},{40,-10},{60,-10}},
            color={255,0,255}),
        Line(points={{-60,-30},{-40,-30},{-40,-50},{20,-50},{20,-30},{60,-30}},
            color={255,0,255}),
        Text(
          extent={{-100,-70},{100,-90}},
          textColor={0,0,0},
          textString="m=%m")}));
end ElectronicCommutator;
