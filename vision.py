from networktables import NetworkTable, NetworkTables

table = NetworkTables.getTable("Limelight")
tx = table.getNumber('tx', None)
ty = table.getNumber('ty', None)
ta = table.getNumber('ta', None)
ts = table.getNumber('ts', None)