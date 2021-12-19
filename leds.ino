void set_all_leds(CRGB color, bool updateLeds = true)
{
  for (int i = 0; i < NUM_LEDS; i++)
  {
    leds[i] = color;
  }

  fill_solid(leds, NUM_LEDS, color);

  if (updateLeds)
  {
    FastLED.show();
  }
}


void clear_leds(bool updateLeds = true)
{
  set_all_leds(CRGB::Black, updateLeds);
}

void set_single_led(int led, CRGB color, bool updateLeds = true)
{
  if(leds[(NUM_LEDS - led) % NUM_LEDS] != color){
    clear_leds(false);

    // Manipulate led order
    leds[(NUM_LEDS - led) % NUM_LEDS] = color;
    Serial.print(" num: ");
    Serial.print((NUM_LEDS - led) % NUM_LEDS);
    if (updateLeds)
    {
      FastLED.show();
    }
  }
}

void update_leds()
  {
    float led = (float)NUM_LEDS*yaw/2/3.141592;    
    set_single_led((int)round(led), CRGB(50,0,0), true);
  }