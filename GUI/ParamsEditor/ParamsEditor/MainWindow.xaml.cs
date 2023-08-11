using Microsoft.Win32;
using System;
using System.IO;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Interop;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Navigation;
using System.Windows.Shapes;
using System.Text.RegularExpressions;

namespace ParamsEditor
{
    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    public partial class MainWindow : Window
    {
        public MainWindow()
        {
            InitializeComponent();
        }

        private void Close_Click(object sender, RoutedEventArgs e)
        {
            this.Close();
        }

        private string params_contents;

        private string SearchValueByName(string str, string value_pattern)
        {
            string result = "";

            var m = Regex.Match(str, ".*(\\b" + value_pattern + "\\b).*");
            if (m.Success)
            {
//                var m2 = Regex.Match(str, "(=\\s*(.*)\\s*;)");
                var m2 = Regex.Match(m.Value, "(= (.*?);(\\r|\\z))");
                if (m2.Success)
                {
                    result = m2.Groups[2].Value;
                }
            }

            return result;
        } 
        private void PopulateRunSection()
        {
            MatchCollection matches = Regex.Matches(params_contents, ".*(prm.run).*", RegexOptions.Multiline);
            foreach (Match match in matches)
            {
                string val = SearchValueByName(match.Value, "PlotSocFlag");
                if (val.Count() > 0)
                    plotSocFlag.IsChecked = (bool)Convert.ChangeType(val, TypeCode.Boolean);

                val = SearchValueByName(match.Value, "PlotItFlag");
                if (val.Count() > 0)
                    plotSocFlag.IsChecked = (bool)Convert.ChangeType(val, TypeCode.Boolean);

                val = SearchValueByName(match.Value, "PlotTempFlag");
                if (val.Count() > 0)
                    plotTempFlag.IsChecked = (bool)Convert.ChangeType(val, TypeCode.Boolean);

                val = SearchValueByName(match.Value, "PlotVFlag");
                if (val.Count() > 0)
                    plotVFlag.IsChecked = (bool)Convert.ChangeType(val, TypeCode.Boolean);

                val = SearchValueByName(match.Value, "PlotIFlag");
                if (val.Count() > 0)
                    plotIFlag.IsChecked = (bool)Convert.ChangeType(val, TypeCode.Boolean);

                val = SearchValueByName(match.Value, "PlotIacsFlag");
                if (val.Count() > 0)
                    plotIacsFlag.IsChecked = (bool)Convert.ChangeType(val, TypeCode.Boolean);

                val = SearchValueByName(match.Value, "MaxTime");
                if (val.Count() > 0)
                    maxTime.Text = val;

                val = SearchValueByName(match.Value, "dt");
                if (val.Count() > 0)
                    dt.Text = val;

                val = SearchValueByName(match.Value, "T2Show");
                if (val.Count() > 0)
                    t2Show.Text = val;

                val = SearchValueByName(match.Value, "Nt");
                if (val.Count() > 0)
                    nt.Text = val;
            }
        }
        private void PopulateSequenceSection()
        {
            int seq_ind = sequence.SelectedIndex + 1;
            MatchCollection matches = Regex.Matches(params_contents, ".*(prm.seq\\(" + seq_ind.ToString() + "\\)).*", RegexOptions.Multiline);
            foreach (Match match in matches)
            {
                string val = SearchValueByName(match.Value, "mod");
                if (val.Count() > 0)
                    seq_mod.Text = val;

                val = SearchValueByName(match.Value, "chr");
                if (val.Count() > 0)
                    seq_chr.Text = val;

                val = SearchValueByName(match.Value, "vth");
                if (val.Count() > 0)
                    seq_vth.Text = val;

                val = SearchValueByName(match.Value, "ins");
                if (val.Count() > 0)
                    seq_ins.Text = val;

                val = SearchValueByName(match.Value, "Nst");
                if (val.Count() > 0)
                    seq_nst.Text = val;

                val = SearchValueByName(match.Value, "swm");
                if (val.Count() > 0)
                    seq_swm.Text = val;
            }
            string prefix = "prm.seq\\(" + seq_ind.ToString() + "\\).tst.";
            matches = Regex.Matches(params_contents, ".*(" + prefix + ").*", RegexOptions.Multiline);
            foreach (Match match in matches)
            {
                string val = SearchValueByName(match.Value, prefix + "ins");
                if (val.Count() > 0)
                    seq_tst_ins.Text = val;

                val = SearchValueByName(match.Value, prefix + "v");
                if (val.Count() > 0)
                    seq_tst_v.IsChecked = (bool)Convert.ChangeType(val, TypeCode.Boolean); ;

                val = SearchValueByName(match.Value, prefix + "i");
                if (val.Count() > 0)
                    seq_tst_i.IsChecked = (bool)Convert.ChangeType(val, TypeCode.Boolean); ;

                val = SearchValueByName(match.Value, prefix + "swm");
                if (val.Count() > 0)
                    seq_tst_swm.Text = val;
            }
        }
        private void PopulateInstrumentSection()
        {
            MatchCollection matches = Regex.Matches(params_contents, ".*(prm.ins).*", RegexOptions.Multiline);
            foreach (Match match in matches)
            {
                string val = SearchValueByName(match.Value, "juntek");
                if (val.Count() > 0)
                    juntek.IsChecked = (bool)Convert.ChangeType(val, TypeCode.Boolean);

                val = SearchValueByName(match.Value, "ka6005p");
                if (val.Count() > 0)
                    ka6005p.IsChecked = (bool)Convert.ChangeType(val, TypeCode.Boolean);

                val = SearchValueByName(match.Value, "kp184");
                if (val.Count() > 0)
                    kp184.IsChecked = (bool)Convert.ChangeType(val, TypeCode.Boolean);

                val = SearchValueByName(match.Value, "swm");
                if (val.Count() > 0)
                    swm.IsChecked = (bool)Convert.ChangeType(val, TypeCode.Boolean);
            }
            matches = Regex.Matches(params_contents, ".*(prm.ins.jun).*", RegexOptions.Multiline);
            foreach (Match match in matches)
            {
                string val = SearchValueByName(match.Value, "ImaxAcDC");
                if (val.Count() > 0)
                    ImaxAcDc.Text = val;

                val = SearchValueByName(match.Value, "minVjuntekInput");
                if (val.Count() > 0)
                    minVjuntekInput.Text = val;

                val = SearchValueByName(match.Value, "juntekEfficencyFactor");
                if (val.Count() > 0)
                    junketEfficencyFactor.Text = val;
            }
        }
        private void PopulateFiles()
        {
            MatchCollection matches = Regex.Matches(params_contents, ".*(prm.files).*", RegexOptions.Multiline);
            foreach (Match match in matches)
            {
                string val = SearchValueByName(match.Value, "saveDir");
                if (val.Count() > 0)
                    saveDir.Text = Regex.Replace(val, "'", "");

                val = SearchValueByName(match.Value, "sufixDir");
                if (val.Count() > 0)
                    sufixDir.Text = Regex.Replace(val, "'", ""); ;

                val = SearchValueByName(match.Value, "prefixFile");
                if (val.Count() > 0)
                    prefixFile.Text = Regex.Replace(val, "'", ""); ;

                val = SearchValueByName(match.Value, "savePath_pIacs");
                if (val.Count() > 0)
                    savePathPiacs.Text = Regex.Replace(val, "'", ""); ;

                val = SearchValueByName(match.Value, "savePath_Rval");
                if (val.Count() > 0)
                    savePathRval.Text = Regex.Replace(val, "'", ""); ;
            }
        }
        private void PopulateKalman()
        {
            string val;
            MatchCollection matches = Regex.Matches(params_contents, ".*(prm.klm).*", RegexOptions.Multiline);
            foreach (Match match in matches)
            {
                val = SearchValueByName(match.Value, "kalmanFlag");
                if (val.Count() > 0)
                    isKalman.IsChecked = (bool)Convert.ChangeType(val, TypeCode.Boolean);
            }

            val = SearchValueByName(params_contents, "BatParamsFile{1");
            if (val.Count() > 0)
                batParamsFile1.Text = Regex.Replace(val, "'", ""); ;

            val = SearchValueByName(params_contents, "BatParamsFile{2");
            if (val.Count() > 0)
                batParamsFile2.Text = Regex.Replace(val, "'", ""); ;
        }
        private void PopulateConfig()
        {
            string val;
            MatchCollection matches = Regex.Matches(params_contents, ".*(prm.cnfg).*", RegexOptions.Multiline);
            foreach (Match match in matches)
            {
                val = SearchValueByName(match.Value, "ToggleFlag");
                if (val.Count() > 0)
                    cfgToggle.IsChecked = (bool)Convert.ChangeType(val, TypeCode.Boolean);
            }

            val = SearchValueByName(params_contents, "Ttoggle");
            if (val.Count() > 0)
                tToggle.Text = val;

            val = SearchValueByName(params_contents, "NtoggleDrop");
            if (val.Count() > 0)
                nToggleDrop.Text = val;

            val = SearchValueByName(params_contents, "minLenIna219");
            if (val.Count() > 0)
                minLenIna219.Text = val;
        }
        private void PopulateBat()
        {
            MatchCollection matches = Regex.Matches(params_contents, ".*(prm.bat).*", RegexOptions.Multiline);
            foreach (Match match in matches)
            {
                string val = SearchValueByName(match.Value, "Vd");
                if (val.Count() > 0)
                    vD.Text = val;

                val = SearchValueByName(match.Value, "ImaxDis");
                if (val.Count() > 0)
                    imaxDis.Text = val;

                val = SearchValueByName(match.Value, "Icharge");
                if (val.Count() > 0)
                    iCharge.Text = val;

                val = SearchValueByName(match.Value, "IchargePhase2");
                if (val.Count() > 0)
                    IchargePhase2.Text = val;

                val = SearchValueByName(match.Value, "IchargePhase3");
                if (val.Count() > 0)
                    IchargePhase3.Text = val;

                val = SearchValueByName(match.Value, "minIphase2");
                if (val.Count() > 0)
                    minIphase2.Text = val;

                val = SearchValueByName(match.Value, "dIphase2");
                if (val.Count() > 0)
                    dIphase2.Text = val;

                val = SearchValueByName(match.Value, "CutOffDisV");
                if (val.Count() > 0)
                    CutOffDisV.Text = val;

                val = SearchValueByName(match.Value, "CutOffChrV");
                if (val.Count() > 0)
                    CutOffChrV.Text = val;
            }
        }
        private void PopulateBoard()
        {
            string val;
            MatchCollection matches = Regex.Matches(params_contents, "^(prm.brd).*", RegexOptions.Multiline);
            foreach (Match match in matches)
            {
                val = SearchValueByName(match.Value, "N_bat1");
                if (val.Count() > 0)
                    nBat1.Text = val;

                val = SearchValueByName(match.Value, "N_bat2");
                if (val.Count() > 0)
                    nBat2.Text = val;

                val = SearchValueByName(match.Value, "spi.rst");
                if (val.Count() > 0)
                    resetTimeout.Text = val;

                val = SearchValueByName(match.Value, "spi.disconnect");
                if (val.Count() > 0)
                    disconnect.Text = val;

                val = SearchValueByName(match.Value, "spi.bypass");
                if (val.Count() > 0)
                    bypass.Text = val;

                val = SearchValueByName(match.Value, "CreatPortFilFlag");
                if (val.Count() > 0)
                    createPortFile.IsChecked = (bool)Convert.ChangeType(val, TypeCode.Boolean);

                val = SearchValueByName(match.Value, "pac.i2cPacAdd");
                if (val.Count() > 0)
                    i2cPacAddr.Text = val;

                val = SearchValueByName(match.Value, "pac.VIpacId");
                if (val.Count() > 0)
                    vIpacId.Text = val;

                val = SearchValueByName(match.Value, "readIpacFlag");
                if (val.Count() > 0)
                    readIpac.IsChecked = (bool)Convert.ChangeType(val, TypeCode.Boolean);

                val = SearchValueByName(match.Value, "pac.Rval");
                if (val.Count() > 0)
                    rVal.Text = val;

                val = SearchValueByName(match.Value, "pac.Rshunt");
                if (val.Count() > 0)
                    rShunt.Text = val;

                val = SearchValueByName(match.Value, "Iacs758Flag");
                if (val.Count() > 0)
                    Iacs758Flag.Text = val;

                val = SearchValueByName(match.Value, "Iacs758Id");
                if (val.Count() > 0)
                    Iacs758Id.Text = val;
            }

            val = SearchValueByName(params_contents, "addrPort");
            if (val.Count() > 0)
                spiPorts.Text = val;

            val = SearchValueByName(params_contents, "bin2port");
            if (val.Count() > 0)
                bin2port.Text = val;
        }
        private void Load_Click(object sender, RoutedEventArgs e)
        {
            // Configure open file dialog box
            var dialog = new Microsoft.Win32.OpenFileDialog();
            dialog.FileName = "createParams"; // Default file name
            dialog.DefaultExt = ".m"; // Default file extension
            dialog.Filter = "M file (.m)|*.m"; // Filter files by extension

            // Show open file dialog box
            bool? result = dialog.ShowDialog();

            // Process open file dialog box results
            if (result == true)
            {
                // Open document
                string filename = dialog.FileName;
                if (File.Exists(filename) == true)
                {
                    params_contents = File.ReadAllText(filename);
                    params_contents = Regex.Replace(params_contents, "%.*\\r", "");

                    // run section
                    PopulateRunSection();

                    // sequence section
                    PopulateSequenceSection();

                    // instrument section
                    PopulateInstrumentSection();

                    // files
                    PopulateFiles();

                    // Kalman
                    PopulateKalman();

                    // board
                    PopulateBoard();

                    // config
                    PopulateConfig();

                    // bat
                    PopulateBat();
                }
            }
        }

        private void Save_Click(object sender, RoutedEventArgs e)
        {
            // Configure save file dialog box
            var dialog = new Microsoft.Win32.SaveFileDialog();
            dialog.DefaultExt = ".m"; // Default file extension
            dialog.Filter = "M file (.m)|*.m"; // Filter files by extension

            // Show save file dialog box
            bool? result = dialog.ShowDialog();

            // Process save file dialog box results
            if (result == true)
            {
                // Save document
                string filename = dialog.FileName;
            }
        }
    }
}
