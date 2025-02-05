package frc.robot.utils;

import edu.wpi.first.wpilibj.I2C;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;

public class ColorSensor {
  protected static final int CMD = 0x80;
  protected static final int MULTI_BYTE_BIT = 0x20;

  protected static final int ENABLE_REGISTER = 0x00;
  protected static final int ATIME_REGISTER = 0x01;
  protected static final int PPULSE_REGISTER = 0x0E;

  protected static final int ID_REGISTER = 0x12;
  protected static final int CDATA_REGISTER = 0x14;
  protected static final int RDATA_REGISTER = 0x16;
  protected static final int GDATA_REGISTER = 0x18;
  protected static final int BDATA_REGISTER = 0x1A;
  protected static final int PDATA_REGISTER = 0x1C;

  protected static final int PON = 0b00000001;
  protected static final int AEN = 0b00000010;
  protected static final int PEN = 0b00000100;
  protected static final int WEN = 0b00001000;
  protected static final int AIEN = 0b00010000;
  protected static final int PIEN = 0b00100000;

  private final double integrationTime = 10;
  private I2C sensor;

  private ByteBuffer buffy = ByteBuffer.allocate(8);

  public short red = 0, green = 0, blue = 0, prox = 0;

  public ColorSensor(I2C.Port port) {
    buffy.order(ByteOrder.LITTLE_ENDIAN);
    sensor = new I2C(port, 0x39); // 0x39 is the address of the Vex ColorSensor V2

    sensor.write(CMD | 0x00, PON | AEN | PEN);

    sensor.write(
        CMD | 0x01,
        (int)
            (256
                - integrationTime
                    / 2.38)); // configures the integration time (time for updating color data)
    sensor.write(CMD | 0x0E, 0b1111);
  }

  public void read() {
    buffy.clear();
    sensor.read(CMD | MULTI_BYTE_BIT | RDATA_REGISTER, 8, buffy);

    red = buffy.getShort(0);
    if (red < 0) {
      red += 0b10000000000000000;
    }

    green = buffy.getShort(2);
    if (green < 0) {
      green += 0b10000000000000000;
    }

    blue = buffy.getShort(4);
    if (blue < 0) {
      blue += 0b10000000000000000;
    }

    prox = buffy.getShort(6);
    if (prox < 0) {
      prox += 0b10000000000000000;
    }
  }

  public int status() {
    buffy.clear();
    sensor.read(CMD | 0x13, 1, buffy);
    return buffy.get(0);
  }

  public void free() {
    sensor.close();
  }
}
