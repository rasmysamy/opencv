package networking;

import edu.wpi.first.wpilibj.networktables.NetworkTable;
import edu.wpi.first.wpilibj.tables.ITable;
import edu.wpi.first.wpilibj.tables.ITableListener;

public class ntClient implements ITableListener {
    public static String ipAddress = "10.35.50.2";
    public static String tableKey = "SmartDashboard";
    NetworkTable m_table;

    public ntClient() {
        NetworkTable.setClientMode();
        NetworkTable.setIPAddress(ipAddress);
        this.m_table = NetworkTable.getTable(tableKey);
        this.m_table.addTableListener(this);
        double[] fakeArray = new double[]{15.0D, 20.0D};
        this.m_table.putNumberArray("targetDerivation", fakeArray);
        this.m_table.putNumber("targetDistance", 420.0D);
        System.out.println(this.m_table.getNumber("targetDistance", 0));
    }
    public void setAngle(double x, double y){
        this.m_table.putNumberArray("targetDerivation", new double[] {x, y});
        this.m_table.putNumber("targetAngle", x);
    }
    public double[] getAngle(){
        double[] defaultArray = {30, 30};
        return this.m_table.getNumberArray("targetDerivation", defaultArray);
    }
    public void valueChanged(ITable itable, String string, Object o, boolean bln) {
        if(true) {
            double[] tempArray = {35.0D, 50.0D};
            this.m_table.putNumberArray("targetDerivation", tempArray);
            this.m_table.putNumber("targetDistance", 20.0D);
        }

    }
}