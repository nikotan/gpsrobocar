
void myPrintln(char* b)
{
  if(useUSB_){
    Serial.println(b);
  }
  if(useXBEE_){
    Serial2.println(b);
  }
}

void myPrint(char* b)
{
  if(useUSB_){
    Serial.print(b);
  }
  if(useXBEE_){
    Serial2.print(b);
  }
}

void myWrite(char b)
{
  if(useUSB_){
    Serial.write(b);
  }
  if(useXBEE_){
    Serial2.write(b);
  }
}

int mySplit(String str, char c, String* arr, int arrSize)
{
  buffString_ = str;
  int cnt = 0;
  int idx = 0;
  while(1){
    idx = buffString_.indexOf(c);
    if(idx == 0){
      buffString_ = buffString_.substring(1);
    } else if(idx > 0) {
      arr[cnt] = buffString_.substring(0, idx);
      buffString_ = buffString_.substring(idx + 1);
      cnt++;
    } else {
      arr[cnt] = buffString_;
      cnt++;
      break;
    }
    if( cnt >= arrSize ){
      break;
    }
  }
  return cnt;
}


void waitSW1()
{
  while(digitalRead(GPS_SW1) == HIGH){
    ;
  }
  delay(GPS_SW1_MSEC);
}


// for debug
// check memory leak
void check_mem() 
{
  uint8_t* stackptr = (uint8_t *)malloc(4); // use stackptr temporarily
  uint8_t* heapptr = stackptr;              // save value of heap pointer
  free(stackptr);      // free up the memory again (sets stackptr to 0)
  stackptr =  (uint8_t *)(SP);              // save value of stack pointer

  sprintf(buff_, "[DEBUG] heap ptr = 0x%08lX , stack ptr = 0x%08lX", (long)heapptr, (long)stackptr);
  myPrintln(buff_);
}

