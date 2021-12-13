within BLDC.UsersGuide;
class ReleaseNotes "Release Notes"
  extends Modelica.Icons.ReleaseNotes;
  annotation (Documentation(info="<html>

<h4>v1.3.3 2021-12-12</h4>
<ul>
<li>Simplified the electronic commutator</li>
<li>(Temporarily) removed HallTimeSpan from DemoEncoder</li>
</ul>

<h4>v1.3.2 2021-12-12</h4>
<p>(Temporarily) included a copy of the SignalPWM from master branch of ModelicaStandardLibrary (availability of triangular carrier).</p>

<h4>v1.3.1 2021-12-12</h4>
<p>Improved electronic commutator working properly for odd number of phases, even number of phases needs further investigation.<br>
   Note that number of phases = power of 2 is not handled by the FundamentalWave machine models.</p>

<h4>v1.3.0 2021-12-12</h4>
<ul>
<li>implemented direction of rotation from hall signals</code></li>
<li>investigate BLDC with number of phases <code>m&gt;3</code></li>
</ul>

<h4>v1.2.0 2021-12-11</h4>
<ul>
<li>improved CommonBlocks.IntervalTest</li>
<li>improved Utilities.ElectronicCommutator3phase</li>
<li>implemented polyphase Utilities.ElectronicCommutator <code>m&gt;3</code></li>
</ul>

<h4>v1.1.0 2021-12-11</h4>
<ul>
<li>corrected angles</li>
<li>under investigation:<br>
 HallTimeSpan: direction of rotation from hall signals<br>
 ElectronicCommutator for number of phases <code>m&gt;3</code></li>
</ul>

<h4>v1.0.0 2021-12-10</h4>
<p>First fully working version</p>

</html>"));
end ReleaseNotes;
