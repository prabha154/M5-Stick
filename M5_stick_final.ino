#include <Arduino.h>
#include <M5StickCPlus2.h>
#include <math.h>
#include <Preferences.h>

Preferences prefs;

// ============================================================
// TARGET STORAGE (7 targets)
// ============================================================
float targetAx[7];
float targetAy[7];
float targetAz[7];

float targetDispPitch[7];
float targetDispRoll[7];

float lockTolerance = 2.0;
float unlockTolerance = 2.5;

const float EMA_ALPHA = 0.2;

float filteredAx = 0;
float filteredAy = 0;
float filteredAz = 0;

float offsetX = 0;
float offsetY = 0;
float offsetZ = 0;

int currentTargetIndex = 0;
int alignmentIndex = -1;

bool justAligned = false;
bool isCurrentlyMatched = false;
bool movedAwayFromPrevious = true;

unsigned long alignedTime = 0;

const int ledPin = 26;

unsigned long firstClickTime = 0;
bool waitingForSecondClick = false;
const unsigned long doubleClickDelay = 400;


// ============================================================
// Normalize vector
// ============================================================
void normalizeVector(float &x, float &y, float &z)
{
    float mag = sqrt(x*x + y*y + z*z);
    if (mag == 0) return;

    x /= mag;
    y /= mag;
    z /= mag;
}


// ============================================================
// Angle between vectors
// ============================================================
float getAngleDeviation(float ax1,float ay1,float az1,
                        float ax2,float ay2,float az2)
{
    float dot=ax1*ax2 + ay1*ay2 + az1*az2;

    if(dot>1) dot=1;
    if(dot<-1) dot=-1;

    return acos(dot)*180.0/PI;
}


// ============================================================
// SAVE TARGETS
// ============================================================
void saveTargets()
{
    prefs.begin("targets",false);

    for(int i=0;i<7;i++)
    {
        prefs.putFloat(("ax"+String(i)).c_str(),targetAx[i]);
        prefs.putFloat(("ay"+String(i)).c_str(),targetAy[i]);
        prefs.putFloat(("az"+String(i)).c_str(),targetAz[i]);

        prefs.putFloat(("p"+String(i)).c_str(),targetDispPitch[i]);
        prefs.putFloat(("r"+String(i)).c_str(),targetDispRoll[i]);
    }

    prefs.end();
}


// ============================================================
// LOAD TARGETS
// ============================================================
void loadTargets()
{
    prefs.begin("targets",true);

    for(int i=0;i<7;i++)
    {
        targetAx[i]=prefs.getFloat(("ax"+String(i)).c_str(),-999);
        targetAy[i]=prefs.getFloat(("ay"+String(i)).c_str(),-999);
        targetAz[i]=prefs.getFloat(("az"+String(i)).c_str(),-999);

        targetDispPitch[i]=prefs.getFloat(("p"+String(i)).c_str(),-999);
        targetDispRoll[i]=prefs.getFloat(("r"+String(i)).c_str(),-999);
    }

    prefs.end();
}


// ============================================================
// BATTERY DISPLAY
// ============================================================
void drawBattery()
{
    int batt = StickCP2.Power.getBatteryLevel();

    StickCP2.Display.setTextColor(GREEN);
    StickCP2.Display.setCursor(200,5);
    StickCP2.Display.printf("%d%%",batt);
}



// ============================================================
// SETUP
// ============================================================
void setup()
{
    auto cfg = M5.config();
    StickCP2.begin(cfg);

    StickCP2.Display.setRotation(1);

    Serial.begin(115200);

    pinMode(ledPin,OUTPUT);
    digitalWrite(ledPin,LOW);

    StickCP2.Display.setTextFont(&fonts::FreeSans9pt7b);
    StickCP2.Display.setTextSize(1);

    loadTargets();

    StickCP2.Display.fillScreen(BLUE);
    StickCP2.Display.setCursor(10,40);
    StickCP2.Display.setTextColor(WHITE);
    StickCP2.Display.println("Calibrating...");
    StickCP2.Display.setCursor(10,60);
    StickCP2.Display.println("Keep Device Still");

    float sumX=0,sumY=0,sumZ=0;

    for(int i=0;i<50;i++)
    {
        float ax,ay,az;

        StickCP2.Imu.getAccel(&ax,&ay,&az);

        sumX+=ax;
        sumY+=ay;
        sumZ+=az;

        delay(10);
    }

    offsetX=sumX/50;
    offsetY=sumY/50;
    offsetZ=(sumZ/50)-1.0;

    float startAx,startAy,startAz;

    StickCP2.Imu.getAccel(&startAx,&startAy,&startAz);

    filteredAx=startAx-offsetX;
    filteredAy=startAy-offsetY;
    filteredAz=startAz-offsetZ;

    normalizeVector(filteredAx,filteredAy,filteredAz);

    StickCP2.Speaker.tone(3000,100);
}



// ============================================================
// LOOP
// ============================================================
void loop()
{
    StickCP2.update();

    float rawAx,rawAy,rawAz;

    StickCP2.Imu.getAccel(&rawAx,&rawAy,&rawAz);

    rawAx-=offsetX;
    rawAy-=offsetY;
    rawAz-=offsetZ;

    filteredAx=(EMA_ALPHA*rawAx)+((1-EMA_ALPHA)*filteredAx);
    filteredAy=(EMA_ALPHA*rawAy)+((1-EMA_ALPHA)*filteredAy);
    filteredAz=(EMA_ALPHA*rawAz)+((1-EMA_ALPHA)*filteredAz);

    float normAx=filteredAx;
    float normAy=filteredAy;
    float normAz=filteredAz;

    normalizeVector(normAx,normAy,normAz);

    // ============================================================
    // PITCH & ROLL CALCULATIONS 
    // ============================================================
    // FIXED: Formulated as independent tilt angles to prevent the 
    // erratic "gimbal lock" effect when pitched steeply upwards or downwards.
    float pitch = atan2(normAx, sqrt(normAy*normAy + normAz*normAz)) * 180.0 / PI;
    float roll  = atan2(normAy, sqrt(normAx*normAx + normAz*normAz)) * 180.0 / PI;


    // ============================================================
    // BUTTON LOGIC
    // ============================================================

    if(StickCP2.BtnA.wasClicked())
    {
        if(!waitingForSecondClick)
        {
            waitingForSecondClick=true;
            firstClickTime=millis();
        }
        else
        {
            waitingForSecondClick=false;

            if(alignmentIndex==-1)
                alignmentIndex=0;
            else
            {
                alignmentIndex=-1;
                justAligned=false;
                isCurrentlyMatched=false;
            }
        }
    }

    if(waitingForSecondClick && (millis()-firstClickTime>doubleClickDelay))
    {
        waitingForSecondClick=false;

        if(alignmentIndex==-1)
        {
            targetAx[currentTargetIndex]=normAx;
            targetAy[currentTargetIndex]=normAy;
            targetAz[currentTargetIndex]=normAz;

            targetDispPitch[currentTargetIndex]=pitch;
            targetDispRoll[currentTargetIndex]=roll;

            saveTargets();

            currentTargetIndex++;

            if(currentTargetIndex>6) currentTargetIndex=0;

            StickCP2.Speaker.tone(2000,100);
        }
    }


    // ============================================================
    // TARGET SCREEN
    // ============================================================

    if (alignmentIndex == -1)
    {
        StickCP2.Display.fillScreen(BLACK);
        StickCP2.Display.setTextColor(GREEN);

        StickCP2.Display.setCursor(5,5);
        StickCP2.Display.printf("P: %.1f R: %.1f", pitch, roll);

        drawBattery();

        for (int i = 0; i < 7; i++)
        {
            int x = (i < 4) ? 5 : 125;
            int y = 25 + ((i % 4) * 20);

            StickCP2.Display.setCursor(x, y);

            if (targetAx[i] == -999)
                StickCP2.Display.printf("T%d: --", i + 1);
            else
                StickCP2.Display.printf("T%d P:%.0f R:%.0f",
                                        i + 1,
                                        targetDispPitch[i],
                                        targetDispRoll[i]);
        }

        digitalWrite(ledPin, HIGH);
    }



    // ============================================================
    // ALIGNMENT MODE 
    // ============================================================
    else if(alignmentIndex>=0 && alignmentIndex<=6)
    {
        float devAngle=getAngleDeviation(normAx,normAy,normAz,
                                         targetAx[alignmentIndex],
                                         targetAy[alignmentIndex],
                                         targetAz[alignmentIndex]);

        if(!movedAwayFromPrevious)
        {
            if(devAngle>unlockTolerance)
                movedAwayFromPrevious=true;
        }

        if(movedAwayFromPrevious && devAngle<=lockTolerance)
        {
            isCurrentlyMatched=true;
        }

        if(isCurrentlyMatched)
        {
            StickCP2.Display.fillScreen(GREEN);
            digitalWrite(ledPin,LOW);

            drawBattery();

            StickCP2.Display.setTextColor(BLACK);

            StickCP2.Display.setCursor(5,10);
            StickCP2.Display.printf("Pitch: %.1f",pitch);

            StickCP2.Display.setCursor(5,25);
            StickCP2.Display.printf("Roll : %.1f",roll);

            StickCP2.Display.setCursor(5,45);
            StickCP2.Display.printf("Target T%d",alignmentIndex+1);

            StickCP2.Display.setCursor(5,65);
            StickCP2.Display.printf("TP:%.0f TR:%.0f",
                                    targetDispPitch[alignmentIndex],
                                    targetDispRoll[alignmentIndex]);

            StickCP2.Display.setCursor(5,95);
            StickCP2.Display.print("ALIGNED!");

            static unsigned long lastBeep=0;
            if(millis()-lastBeep>300)
            {
                StickCP2.Speaker.tone(4000,50);
                lastBeep=millis();
            }

            if(!justAligned)
            {
                justAligned=true;
                alignedTime=millis();
            }

            if(millis()-alignedTime>2000)
            {
                alignmentIndex++;
                justAligned=false;
                isCurrentlyMatched=false;
                movedAwayFromPrevious=false;
            }
        }
        else
        {
            StickCP2.Display.fillScreen(YELLOW);
            digitalWrite(ledPin,HIGH);

            drawBattery();

            StickCP2.Display.setTextColor(BLACK);

            StickCP2.Display.setCursor(5,10);
            StickCP2.Display.printf("Pitch: %.1f",pitch);

            StickCP2.Display.setCursor(5,25);
            StickCP2.Display.printf("Roll : %.1f",roll);

            StickCP2.Display.setCursor(5,45);
            StickCP2.Display.printf("Target T%d",alignmentIndex+1);

            StickCP2.Display.setCursor(5,65);
            StickCP2.Display.printf("TP:%.0f TR:%.0f",
                                    targetDispPitch[alignmentIndex],
                                    targetDispRoll[alignmentIndex]);

            StickCP2.Display.setCursor(5,95);
            StickCP2.Display.print("MISMATCH");
        }
    }



    // ============================================================
    // DONE SCREEN
    // ============================================================

    else if(alignmentIndex==7)
    {
        StickCP2.Display.fillScreen(BLUE);

        StickCP2.Display.setTextColor(WHITE);
        StickCP2.Display.setCursor(20,40);
        StickCP2.Display.setTextSize(2);
        StickCP2.Display.print("ALL DONE");

        StickCP2.Display.setTextSize(1);
        StickCP2.Display.setCursor(10,80);
        StickCP2.Display.print("Db-Click to Exit");
    }

    delay(50);
}
