#ifndef __DRIVERS_USB_DWC2_JZ4780_H
#define __DRIVERS_USB_DWC2_JZ4780_H

void jz4780_set_charger_current(struct dwc2 *dwc);
void jz4780_set_vbus(struct dwc2 *dwc, int is_on);
int dwc2_get_id_level(struct dwc2 *dwc);
void dwc2_input_report_power2_key(struct dwc2 *dwc);

#endif	/* __DRIVERS_USB_DWC2_JZ4780_H */
