within BLDC.ToMSL;
block IntervalTest
  "Output y is true, if input u is within the specified interval"
   extends Modelica.Blocks.Interfaces.partialBooleanSO;
  parameter Boolean useConstantLimits=true "Otherwise dynamic inputs" annotation(Evaluate=true);
  parameter Real constantLowerLimit "Lower limit of interval" annotation(Dialog(enable=useConstantLimits));
  parameter Boolean ClosedOnLeft=false "Include lower limit?" annotation(Evaluate=true);
  parameter Real constantUpperLimit "Upper limit of interval" annotation(Dialog(enable=useConstantLimits));
  parameter Boolean ClosedOnRight=false "Include upper limit?" annotation(Evaluate=true);
  parameter Boolean InsideInterval=true "u inside interval?" annotation(Evaluate=true);
  Modelica.Blocks.Interfaces.RealInput u annotation (Placement(transformation(
          extent={{-140,-20},{-100,20}}), iconTransformation(extent={{-140,-20},{
            -100,20}})));
  Modelica.Blocks.Interfaces.RealInput lowerLimit if not useConstantLimits
    annotation (Placement(transformation(extent={{-140,-100},{-100,-60}})));
  Modelica.Blocks.Interfaces.RealInput upperLimit if not useConstantLimits
    annotation (Placement(transformation(extent={{-140,60},{-100,100}})));
protected
  Modelica.Blocks.Interfaces.RealInput lowerLimitInternal annotation (Placement(
        transformation(extent={{-94,-84},{-86,-76}}), iconTransformation(extent={
            {-94,56},{-86,64}})));
  Modelica.Blocks.Interfaces.RealInput upperLimitInternal annotation (Placement(
        transformation(extent={{-94,76},{-86,84}}), iconTransformation(extent={{-94,76},
            {-86,84}})));
equation
  if useConstantLimits then
    upperLimitInternal = constantUpperLimit;
    lowerLimitInternal = constantLowerLimit;
  else
    connect(upperLimitInternal, upperLimit);
    connect(lowerLimitInternal, lowerLimit);
  end if;
  assert(upperLimitInternal>lowerLimitInternal,"Erroneous interval limits");
  if ClosedOnLeft then
    if ClosedOnRight then
      if InsideInterval then
        y = u>=lowerLimitInternal and u<=upperLimitInternal;
      else
        y = u< lowerLimitInternal or  u> upperLimitInternal;
      end if;
    else
      if InsideInterval then
        y = u>=lowerLimitInternal and u< upperLimitInternal;
      else
        y = u< lowerLimitInternal or  u>=upperLimitInternal;
      end if;
    end if;
  else
    if ClosedOnRight then
      if InsideInterval then
        y = u> lowerLimitInternal and u<=upperLimitInternal;
      else
        y = u<=lowerLimitInternal or  u> upperLimitInternal;
      end if;
    else
      if InsideInterval then
        y = u> lowerLimitInternal and u< upperLimitInternal;
      else
        y = u<=lowerLimitInternal or  u>=upperLimitInternal;
      end if;
    end if;
  end if;
  annotation (Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},
            {100,100}}), graphics={Text(
          extent={{-60,50},{60,-30}},
          textColor={0,0,0},
          textString="[ ? ]"),
        Line(visible=not useConstantLimits, points={{-100,-80},{-50,-80},{-50,-26}}, color={0,0,0}),
        Line(visible=not useConstantLimits, points={{-100,80},{50,80},{50,34}}, color={0,0,0})}),
                            Documentation(info="<html>
<p>
The output is <strong>true</strong> if the Real input is within interval specified by lower and upper limit.
The Boolean parameters ClosedOnLeft and ClosedOnRight indicate whether lower respectively upper limit are included in the interval. 
If Boolean parameter InsideInterval = false, the output is inverted (u is outside the interval).
</p>
</html>"));
end IntervalTest;
