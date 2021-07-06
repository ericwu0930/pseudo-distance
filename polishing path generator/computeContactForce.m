function [contactForce]=computeContactForce(maxContactDepth,surfaceLocalAlpha,inContact)
%% force sensor points
global E; %modulus /MPa
global Beta; %power index
global H;  %disc thickness /m
global toolSamplePoints;
global toolWeightArea;
newLocalAlpha=surfaceLocalAlpha+maxContactDepth*[1 0 0]';
contactDepths=zeros(length(toolSamplePoints),1);
for i=1:length(toolSamplePoints)
    contactDepths(i)= [1 toolSamplePoints(i,1) toolSamplePoints(i,2)]*newLocalAlpha(:,i);  % workpiece-disc bottom point dist
end
contactPressure=E*(contactDepths/H).^Beta.*(contactDepths>=0).*(contactDepths<H).*(inContact);
contactForce=toolWeightArea'*contactPressure;