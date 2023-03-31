//
//  settings.h
//  CNC
//
//  Created by Ruedi Heimlicher on 30.03.2023.
//

#ifndef settings_h
#define settings_h

#define HIGH 1
#define LOW 0
// Ramp
#define RAMP_OK      1 // Ramp einschalten
#define RAMPFAKTOR   2 // Verlaengerung der delayzeit
#define RAMPZEIT     800 // Mindestdauer fuer Ramp

#define RAMPSTARTBIT 1
#define RAMPENDBIT 2
#define RAMPEND0BIT 3 // Beginn der Endrampe
#define RAMPOKBIT    7
#define RAMPSCHRITT  1

#define TIMER_ON           1 // Bit fuer timerfunktion start



#define DC_PWM              22

#define TIMERINTERVALL 128
#endif /* settings_h */
