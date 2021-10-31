#include "SDcard.hpp"
#include "main.h"
namespace EZ
{
  namespace SD
  {
    void
    update_auto_sd() {
      // If no SD card, return
      if (!IS_SD_CARD) return;

      FILE* usd_file_write = fopen("/usd/auto.txt", "w");
      std::string cp_str = std::to_string(EZ::autoSelector.CurrentAutonPage);
      char const *cp_c = cp_str.c_str();
      fputs(cp_c, usd_file_write);
      fclose(usd_file_write);
    }

    void
    init_auto_sd() {
      // If no SD card, return
      if (!IS_SD_CARD)  return;

      // Auton Selector
      FILE* usd_file_read = fopen("/usd/auto.txt", "r");
      char buf[5];
      fread(buf, 1, 5, usd_file_read);
      EZ::autoSelector.CurrentAutonPage = std::stoi(buf);
      fclose(usd_file_read);

      if(EZ::autoSelector.CurrentAutonPage>EZ::autoSelector.AutonCount-1 || EZ::autoSelector.CurrentAutonPage<0)
      {
        EZ::autoSelector.CurrentAutonPage=0;
        update_auto_sd();
      }
    }
  }
}
