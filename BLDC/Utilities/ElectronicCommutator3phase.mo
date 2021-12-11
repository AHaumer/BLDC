within BLDC.Utilities;
block ElectronicCommutator3phase "Electronic commutator for 3 phases"
  extends BLDC.BaseBlocks.BaseElectronicCommutator(final m=3,
    final orientation);
protected
  constant Integer I1[6]={2,3,2,1,1,3} "Indices of on states in sectors";
  constant Integer I0[6]={3,1,1,2,3,2} "Indices of off states in sectors";
  constant Integer Ix[6]={1,2,3,3,2,1} "Indices of open states in sectors";
  //Hall 100 110 010 011 001 101
  //Fire x10 01x 0x1 x01 10x 1x0
  Integer c=sum({if uC[k] then integer(2^(k - 1)) else 0 for k in 1:m}) "Sector code";
algorithm
  assert(c>=1 and c<=6, "Hall sensor sector error!");
  fire_p[I1[c]]:=internalPWM;
  fire_n[I1[c]]:=not internalPWM;
  fire_p[I0[c]]:=not internalPWM;
  fire_n[I0[c]]:=internalPWM;
  fire_p[Ix[c]]:=false;
  fire_n[Ix[c]]:=false;
  annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
        Line(points={{-60,30},{-60,50},{0,50},{0,30},{60,30},{60,50}}, color={255,
              0,255}),
        Line(points={{-60,-10},{-20,-10},{-20,10},{40,10},{40,-10},{60,-10}},
            color={255,0,255}),
        Line(points={{-60,-30},{-40,-30},{-40,-50},{20,-50},{20,-30},{60,-30}},
            color={255,0,255})}),                                Diagram(
        coordinateSystem(preserveAspectRatio=false)),
    Documentation(info="<html>
<p>
According to the Hall sensor signals <code>uC[m=3]</code>, the inactive (open) leg is choossen 
as well as the two legs that apply the DC voltage to two terminals of the machine, 
and therefore let a current flow through the corresponding phases of the machine 
to obtain a current space phasor perpendicular to the rotor's magnetic field. 
Of course due to the block commutation the angle between rotor position and current space phasor 
varies in the range <code>[&#177;90&deg; - 30&deg;, &#177;90&deg; + 30&deg;]</code>.
</p>
<p>
Setting the input <code>pwm = true</code> chooses the position of the current space phasor as <code>+90&deg;</code>, whereas 
setting the input <code>pwm = false</code> results in a position of the current space phasor as <code>-90&deg;</code> w.r.t. the rotor position. 
Applying a pwm signal (as for a brushed DC machine) to the input <code>pwm</code> offers the possibility to set the mean voltage applied to the phases choosen according to the Hall signals.
</p>
</html>"));
end ElectronicCommutator3phase;
