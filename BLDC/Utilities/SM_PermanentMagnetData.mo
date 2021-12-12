within BLDC.Utilities;
record SM_PermanentMagnetData
  "Common parameters for synchronous machines with permanent magnet"
  import Modelica.Constants.pi;
  parameter Integer ms(min=3) = 3 "Number of stator phases";
  parameter Integer mSystems=Modelica.Electrical.Polyphase.Functions.numberOfSymmetricBaseSystems(ms)
    "Number of symmetric base systems" annotation(Dialog(enable=false));
  parameter Integer mBasic=integer(ms/mSystems) "Number of phases of basic systems" annotation(Dialog(enable=false));
  extends
    Modelica.Electrical.Machines.Utilities.ParameterRecords.SM_PermanentMagnetData(
    VsOpenCircuit=100,
    Rs=ms/3*0.03,
    Lssigma=ms/3*0.1/(2*pi*fsNominal),
    Lmd=ms/3*0.3/(2*pi*fsNominal),
    Lmq=ms/3*0.3/(2*pi*fsNominal),
    useDamperCage=false,
    Lrsigmad=ms/3*0.05/(2*pi*fsNominal),
    Rrd=ms/3*0.04);
  annotation (
    defaultComponentName="smpmData",
    defaultComponentPrefixes="parameter",
    Documentation(info="<html>
<p>Basic parameters of synchronous machines with permanent magnet are predefined with default values.</p>
</html>"));
end SM_PermanentMagnetData;
