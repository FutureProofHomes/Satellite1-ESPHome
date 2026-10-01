import { Theme } from './settings/types';
import { Satellite1WebUI } from './components/generated/Satellite1WebUI';

let theme: Theme = 'light';

function App() {
  function setTheme(theme: Theme) {
    if (theme === 'dark') {
      document.documentElement.classList.add('dark');
    } else {
      document.documentElement.classList.remove('dark');
    }
  }

  setTheme(theme);

  return (
    <>
      <Satellite1WebUI />
    </>
  ); // %EXPORT_STATEMENT%
}

export default App;
