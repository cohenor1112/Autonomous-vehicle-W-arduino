//in case we want to fine-tune this program (change transmmiting frequency) , the only variable we need to change is "i" in "for loop"

#define delay_38khz 10

const int SIG_TRANSMMIT_PIN = 8;

void setup()
{
  pinMode(SIG_TRANSMMIT_PIN, OUTPUT);
  digitalWrite(SIG_TRANSMMIT_PIN, LOW);
}

void loop()
{
  for (unsigned int i = 0; i < 12; i++)           //i=20 is for +-950hz (in the reciever with timeout=2500 - its between 990-1100hz.
  {
    digitalWrite(SIG_TRANSMMIT_PIN, HIGH);
    delayMicroseconds(delay_38khz);
    digitalWrite(SIG_TRANSMMIT_PIN, LOW);
    delayMicroseconds(delay_38khz);
  }
  for (unsigned int i = 0; i < 12; i++)
  {
    digitalWrite(SIG_TRANSMMIT_PIN, LOW);
    delayMicroseconds(delay_38khz);
    digitalWrite(SIG_TRANSMMIT_PIN, LOW);
    delayMicroseconds(delay_38khz);
  }
}
