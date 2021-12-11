within BLDC.CommonBlocks;
block IntervalTest
  "Output y is true, if input u is within the specified interval"
  extends Modelica.Blocks.Interfaces.partialBooleanThresholdComparison(final threshold=0);
  parameter Real lowerLimit "Lower limit of interval";
  parameter Boolean ClosedOnLeft=false "Include lower limit?" annotation(Evaluate=true);
  parameter Real upperLimit "Upper limit of interval";
  parameter Boolean ClosedOnRight=false "Include upper limit?" annotation(Evaluate=true);
  parameter Boolean InsideInterval=true "u inside interval?" annotation(Evaluate=true);
equation
  if ClosedOnLeft then
    if ClosedOnRight then
      if InsideInterval then
        y = u>=lowerLimit and u<=upperLimit;
      else
        y = u< lowerLimit or  u> upperLimit;
      end if;
    else
      if InsideInterval then
        y = u>=lowerLimit and u< upperLimit;
      else
        y = u< lowerLimit or  u>=upperLimit;
      end if;
    end if;
  else
    if ClosedOnRight then
      if InsideInterval then
        y = u> lowerLimit and u<=upperLimit;
      else
        y = u<=lowerLimit or  u> upperLimit;
      end if;
    else
      if InsideInterval then
        y = u> lowerLimit and u< upperLimit;
      else
        y = u<=lowerLimit or  u>=upperLimit;
      end if;
    end if;
  end if;
  annotation (Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},
            {100,100}}), graphics={Text(
          extent={{-60,50},{60,-30}},
          textColor={0,0,0},
          textString="[ ? ]")}),
                            Documentation(info="<html>
<p>
The output is <strong>true</strong> if the Real input is within interval specified by lower and upper limit.
The Boolean parameters ClosedOnLeft and ClosedOnRight indicate whether lower respectively upper limit are included in the interval. 
If Boolean parameter InsideInterval = false, the output is inverted (u is outside the interval).
</p>
</html>"));
end IntervalTest;
