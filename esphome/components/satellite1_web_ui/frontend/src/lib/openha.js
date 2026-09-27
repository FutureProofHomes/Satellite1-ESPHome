/**
 * The Open Home Assistant tap, shared by the fix drawer (splash.jsx) and the onboarding wizard
 * (setup.jsx). On phones it opens the companion app through its homeassistant:// scheme - one hop,
 * no interstitial tab, no internet needed, and notably the one kind of link that escapes the iOS
 * captive-portal sheet. Desktops skip the intercept and let the anchor open its real href (the My
 * Home Assistant web redirect) in a new tab.
 *
 * Deliberately NO automatic web fallback anymore (owner report, September 26 2026). The old
 * version armed a timer that judged "no app claimed the tap" off the page still being visible -
 * but iOS shows an "Open in Home Assistant?" confirmation dialog for custom schemes, and the page
 * stays visible for as long as the dialog stands, so the timer routinely fired mid-dialog and
 * walked the tab to my.home-assistant.io underneath a switch that was actually succeeding. The
 * customer came back from the app to a hijacked tab. No timer survives a dialog of arbitrary
 * length, so the fallback is a person's decision now: both callers render a visible "open it in
 * your browser instead" link beside the button, and a phone without the app (whose tap gets the
 * OS's cannot-open notice) has it right there.
 *
 * Parameterised by target, though both callers - the fix drawer and the wizard - point at the same
 * landing now: Settings -> Devices & Services (owner decision, September 26 2026; the fix drawer's
 * old deep-link into the ESPHome integration page read as being dumped somewhere unexpected).
 */
export function openHomeAssistant(e, appUrl) {
  if (!/iphone|ipad|ipod|android/i.test(navigator.userAgent)) return;
  e.preventDefault();
  location.href = appUrl;
}
