Gcode=fopen('ToolPos0515-final.txt','w');
for i=1:length(globalPos)
    if globalFlag(i)==0
        fprintf(Gcode,'MoveL [[%8.4f,%8.4f,%8.4f],[%6.4f,%6.4f,%6.4f,%6.4f],[-1,0,-2,0],[9E9,9E9,9E9,9E9,9E9,9E9]],[%8.4f,500,5000,1000],z10,toolsj\\WObj:=shangjiao;\n',globalPos(i,:),globalQuater(i,:),0.5*globalV(i));
    elseif globalFlag(i)==-1
        fprintf(Gcode,'MoveL [[%8.4f,%8.4f,%8.4f],[%6.4f,%6.4f,%6.4f,%6.4f],[-1,0,-2,0],[9E9,9E9,9E9,9E9,9E9,9E9]],[%8.4f,500,5000,1000],z10,toolsj\\WObj:=shangjiao;\n',globalPos(i,:),globalQuater(i,:),0.5*globalV(i));
    elseif globalFlag(i)==1
        fprintf(Gcode,'MoveL [[%8.4f,%8.4f,%8.4f],[%6.4f,%6.4f,%6.4f,%6.4f],[-1,0,-2,0],[9E9,9E9,9E9,9E9,9E9,9E9]],[%8.4f,500,5000,1000],z10,toolsj\\WObj:=shangjiao;\n',globalPos(i,:),globalQuater(i,:),0.5*globalV(i));
    elseif globalFlag(i)==-2
        fprintf(Gcode,'MoveL [[%8.4f,%8.4f,%8.4f],[%6.4f,%6.4f,%6.4f,%6.4f],[-1,0,-2,0],[9E9,9E9,9E9,9E9,9E9,9E9]],[%8.4f,500,5000,1000],z10,toolsj\\WObj:=shangjiao;\n',globalPos(i,:),globalQuater(i,:),0.5*globalV(i));
        fprintf(Gcode,'SetDO\\Sync, DO10_3, 0;\n');
    elseif globalFlag(i)==2
        fprintf(Gcode,'MoveL [[%8.4f,%8.4f,%8.4f],[%6.4f,%6.4f,%6.4f,%6.4f],[-1,0,-2,0],[9E9,9E9,9E9,9E9,9E9,9E9]],[%8.4f,500,5000,1000],z0,toolsj\\WObj:=shangjiao;\n',globalPos(i,:),globalQuater(i,:),5);
        fprintf(Gcode,'SetDO\\Sync, DO10_3, 1;\n');
    elseif globalFlag(i)==-3
        fprintf(Gcode,'MoveL [[%8.4f,%8.4f,%8.4f],[%6.4f,%6.4f,%6.4f,%6.4f],[-1,0,-2,0],[9E9,9E9,9E9,9E9,9E9,9E9]],[%8.4f,500,5000,1000],z10,toolsj\\WObj:=shangjiao;\n',globalPos(i,:),globalQuater(i,:),0.5*globalV(i));
        fprintf(Gcode,'SetDO\\Sync, DO10_3, 0;\n');
        fprintf(Gcode,'FCstatus :=0;\n');
        fprintf(Gcode,'MoveL [[%8.4f,%8.4f,%8.4f],[%6.4f,%6.4f,%6.4f,%6.4f],[-1,0,-2,0],[9E9,9E9,9E9,9E9,9E9,9E9]],[%8.4f,500,5000,1000],z10,toolsj\\WObj:=shangjiao;\n',globalPos(i,:)+[0 0 10],globalQuater(i,:),5);
        fprintf(Gcode,'MoveL [[%8.4f,%8.4f,%8.4f],[%6.4f,%6.4f,%6.4f,%6.4f],[-1,0,-2,0],[9E9,9E9,9E9,9E9,9E9,9E9]],[%8.4f,500,5000,1000],z10,toolsj\\WObj:=shangjiao;\n',globalPos(i,:)+[0 0 60],globalQuater(i,:),10);
    elseif globalFlag(i)==3
        fprintf(Gcode,'MoveL [[%8.4f,%8.4f,%8.4f],[%6.4f,%6.4f,%6.4f,%6.4f],[-1,0,-2,0],[9E9,9E9,9E9,9E9,9E9,9E9]],[%8.4f,500,5000,1000],z10,toolsj\\WObj:=shangjiao;\n',globalPos(i,:)+[0 0 60],globalQuater(i,:),20);
        fprintf(Gcode,'MoveL [[%8.4f,%8.4f,%8.4f],[%6.4f,%6.4f,%6.4f,%6.4f],[-1,0,-2,0],[9E9,9E9,9E9,9E9,9E9,9E9]],[%8.4f,500,5000,1000],z10,toolsj\\WObj:=shangjiao;\n',globalPos(i,:)+[0 0 10],globalQuater(i,:),10);
        fprintf(Gcode,'MoveL [[%8.4f,%8.4f,%8.4f],[%6.4f,%6.4f,%6.4f,%6.4f],[-1,0,-2,0],[9E9,9E9,9E9,9E9,9E9,9E9]],[%8.4f,500,5000,1000],z0,toolsj\\WObj:=shangjiao;\n',globalPos(i,:),globalQuater(i,:),3); 
        fprintf(Gcode,'FCstatus :=1;\n');
        fprintf(Gc ode,'SetDO\\Sync, DO10_3, 1;\n');
    elseif globalFlag(i)==5
        fprintf(Gcode,'MoveL [[%8.4f,%8.4f,%8.4f],[%6.4f,%6.4f,%6.4f,%6.4f],[-1,0,-2,0],[9E9,9E9,9E9,9E9,9E9,9E9]],[%8.4f,500,5000,1000],z10,toolsj\\WObj:=shangjiao;\n',globalPos(i,:),globalQuater(i,:),50);
    end
end
fclose(Gcode);
