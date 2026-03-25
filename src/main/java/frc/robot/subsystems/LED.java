// package frc.robot.subsystems;

// import edu.wpi.first.wpilibj.AddressableLED;
// import edu.wpi.first.wpilibj.AddressableLEDBuffer;
// import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj.DriverStation.Alliance;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;

// public class LED extends SubsystemBase {
//   /** Creates a new LED. */
//   private int fade = 0;
//   private int direction = 0;
//   private AddressableLED m_led0 = new AddressableLED(0);
//   private AddressableLEDBuffer m_ledBuffer0 = new AddressableLEDBuffer(864);

//   private int i = 4;
//   private int flashtimer = 0;
//   public int time = 0;
//   public boolean isEnabled = false;
//   public boolean intakeNote = false;
//   public boolean shootReady = false;
//   public boolean climbTime = false;

//   public boolean flashDone = false;
//   private int flashcount = 0;
//   private int m_rainbowFirstPixelHue;

//   public LED() {
//     m_led0.setLength(m_ledBuffer0.getLength());
//     m_led0.start();
//    }

//   @Override
//   public void periodic() {
    

//     if (isEnabled == false) {
//         i++;
//         m_ledBuffer0.setRGB(i-4,0,0,0);
//         if(DriverStation.getAlliance().isPresent()){
//           if(DriverStation.getAlliance().get() == Alliance.Blue){
//             m_ledBuffer0.setRGB(i, 0, 0, 100);
//           } else{
//             m_ledBuffer0.setRGB(i, 100, 0, 0);
//           }
//         }
//         if( i >= 863){
//           m_ledBuffer0.setRGB(860, 0,0,0);
//           m_ledBuffer0.setRGB(861, 0,0,0);
//           m_ledBuffer0.setRGB(862, 0,0,0);
//           m_ledBuffer0.setRGB(863, 0,0,0);
//           i = 4;
//         }

//       m_led0.setData(m_ledBuffer0);
//     } else{
//       if(climbTime){
//         rainbow();
//         m_led0.setData(m_ledBuffer0);
//       } else if(intakeNote && !flashDone){
//         if(flashcount%2 == 1){
//           for (var i = 0; i < m_ledBuffer0.getLength(); i = i + 1) {
//               m_ledBuffer0.setRGB(i, 130,130,130);
//           }
//         } else{
//             for (var i = 0; i < m_ledBuffer0.getLength(); i = i + 1) {
//               m_ledBuffer0.setRGB(i, 0,0,0);
//           }
//         }
//           flashtimer++;
//           if(flashtimer > 10){
//             flashtimer = 0;
//             flashcount++;
//           }
//           if(flashcount == 5){
//             flashcount = 0;
//             flashtimer = 0;
//             flashDone = true;
//           }
//           m_led0.setData(m_ledBuffer0);
//       } else if(shootReady && !flashDone){
//           for (var i = 0; i < m_ledBuffer0.getLength(); i = i + 1) {
//               m_ledBuffer0.setRGB(i, 130,130,130);
//           }
//           flashtimer++;
//           if(flashtimer > 10){
//             flashtimer = 0;
//             flashcount++;
//           }
//           if(flashcount == 5){
//             flashDone = true;
//           }
//           m_led0.setData(m_ledBuffer0);
//       } else {
//         if(DriverStation.getAlliance().isPresent()){
//           if(DriverStation.getAlliance().get() == Alliance.Blue){
//             for (var i = 0; i < m_ledBuffer0.getLength(); i = i + 1) {
//               m_ledBuffer0.setRGB(i, 0, 0, 130);
//             }
//           }else{
//             if(DriverStation.getAlliance().get() == Alliance.Red){
//               for (var i = 0; i < m_ledBuffer0.getLength(); i = i + 1) {
//                 m_ledBuffer0.setRGB(i, 130, 0, 0);
//               }
//             }
//           }
//         }
//         m_led0.setData(m_ledBuffer0);
//       }
//     }
//   }


//   private void rainbow() {
//     // For every pixel
//     for (var i = 0; i < m_ledBuffer0.getLength(); i = i + 1) {
//       // Calculate the hue - hue is easier for rainbows because the color
//       // shape is a circle so only one value needs to precess
//       final var hue = (m_rainbowFirstPixelHue + (i * 180 / m_ledBuffer0.getLength())) % 180;
//       // Set the value
//       m_ledBuffer0.setHSV(i, hue, 255, 128);

//     }
//     // Increase by to make the rainbow "move"
//     m_rainbowFirstPixelHue += 2;
//     // Check bounds
//     m_rainbowFirstPixelHue %= 180;
//   }

// }