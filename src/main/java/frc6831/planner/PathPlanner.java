package frc6831.planner;

import net.sourceforge.argparse4j.ArgumentParsers;
import net.sourceforge.argparse4j.inf.ArgumentParser;
import net.sourceforge.argparse4j.inf.ArgumentParserException;
import net.sourceforge.argparse4j.inf.Namespace;
import org.a05annex.util.Utl;
import org.jetbrains.annotations.NotNull;

import javax.swing.*;
import javax.swing.event.MenuEvent;
import javax.swing.event.MenuListener;
import javax.swing.filechooser.FileNameExtensionFilter;
import java.awt.*;
import java.awt.event.*;
import java.io.File;

public class PathPlanner extends JFrame implements ActionListener, MenuListener, WindowListener, TitleChangeListener {

    private int m_nSizeX = 565;         // the initial X size of the app window (for images for class website).
    private int m_nSizeY = 574;         // the initial Y size of the app window (for images for class website).
    private final GraphicsConfiguration m_graphicsConfig;       // the graphics configuration of the window device

    //Context menu items
    private final JMenuItem m_menuContextLoadField;  // the menu file-load field button
    private final JMenuItem m_menuContextLoadRobot;  // the menu file-load robot button

    // Path Menu items
    private final JMenu m_menuPath;
    // - file IO for the path
    private final JMenuItem m_menuPathNewPath;          // the menu start a new path
    private final JMenuItem m_menuPathLoadPath;         // the menu file-load path button
    private final JMenuItem m_menuPathReloadPath;       // the menu file-reload path button
    private final JMenuItem m_menuPathSavePath;         // the menu path save button
    private final JMenuItem m_menuPathSaveAsPath;       // the menu path save-as button
    // - animating the path
    private final JMenuItem m_menuPathPlay;             // play an animation of the current path
    private final JMenuItem m_menuSpeedMultiplier;      // globally change the speed of a path
    // - path - switch alliance - in case you drew the path at the wrong end.
    private final JMenuItem m_menuSwitchAlliance;       // switch the alliance for the path
    // - path clear and start again - clearing and starting a new path (severe)
    private final JMenuItem m_menuPathClearPath;        // the menu edit - clear the current path and reset to

    private String m_userDir;
    private String m_fieldResourceDir;
    private String m_robotResourceDir;
    private String m_pathResourceDir;
    private final PathCanvas m_canvas;                  // the rendering canvas (defined at the end of this file)
    private final Robot m_robot = new Robot();          // the robot description
    private final Field m_field = new Field();          // the field description

    public static void main(@NotNull final String[] args) {
        // Setup the commandline argument parser and parse any commandline arguments
        ArgumentParser parser = ArgumentParsers.newFor("PathPlanner").build()
                .description("Swerve Drive Path Planner");
        parser.addArgument("-r", "--robot")
                .type(String.class)
                .help("specify a field description file");
        parser.addArgument("-f", "--field")
                .type(String.class)
                .help("specify a robot description file");
        String robotDescFile = null;
        String fieldDescFile = null;
        try {
            Namespace parsedArgs = parser.parseArgs(args);
            robotDescFile = parsedArgs.get("robot");
            fieldDescFile = parsedArgs.get("field");
        } catch (ArgumentParserException e) {
            parser.handleError(e);
        }
        // start the path planning window
        try {
            final PathPlanner pathPlanner = new PathPlanner(robotDescFile, fieldDescFile);
            pathPlanner.setVisible(true);
        } catch (final Throwable t) {
            t.printStackTrace();
            System.exit(0);
        }
    }

    private PathPlanner(String robotDescFile, String fieldDescFile) {
        //------------------------------------------------------------------
        // setup the window for drawing the field and paths
        //------------------------------------------------------------------
        // resize the default if it doesn't fit in the full screen
        m_graphicsConfig = getGraphicsConfiguration();
        final Rectangle boundsRect = m_graphicsConfig.getBounds();
        if (boundsRect.width < m_nSizeX) m_nSizeX = boundsRect.width;
        if (boundsRect.height < m_nSizeY) m_nSizeY = boundsRect.height;
        setSize(m_nSizeX, m_nSizeY);

        //------------------------------------------------------------------
        // Setup the default resource directories
        //------------------------------------------------------------------
        m_userDir = System.getProperty("user.dir");
        m_fieldResourceDir = m_userDir + "/resources/fields";
        m_robotResourceDir = m_userDir + "/resources/robots";
        m_pathResourceDir = m_userDir + "/resources/paths/2025";

        // ----------------------------------------------------------------------------------
        // Create the menu with exit
        // ----------------------------------------------------------------------------------
        final JMenuBar menubar = new JMenuBar();

        final JMenu menuContext = createMenu(menubar, "Context");
        m_menuContextLoadField = createMenuItem(menuContext, "Load Game Field ...", this);
        m_menuContextLoadRobot = createMenuItem(menuContext, "Load Robot ...", this);

        m_menuPath = createMenu(menubar, "Path");
        m_menuPath.addMenuListener(this);
        m_menuPathNewPath = createMenuItem(m_menuPath, "New ...", this);
        m_menuPathLoadPath = createMenuItem(m_menuPath, "Load ...", this);
        m_menuPathReloadPath = createMenuItem(m_menuPath, "Reload", this);
        m_menuPathSavePath = createMenuItem(m_menuPath, "Save", this);
        m_menuPathSaveAsPath = createMenuItem(m_menuPath, "Save As ...", this);
        m_menuPath.addSeparator();
        m_menuPathPlay = createMenuItem(m_menuPath, "Play Path", this);
        m_menuSpeedMultiplier = createMenuItem(m_menuPath, "Speed Multiplier", this);
        m_menuPath.addSeparator();
        m_menuSwitchAlliance = createMenuItem(m_menuPath, "Switch Alliance", this);
        m_menuPath.addSeparator();
        m_menuPathClearPath = createMenuItem(m_menuPath, "Clear Path", this);

        // the menubar is configured, now add it
        setJMenuBar(menubar);
        // and right now everything is so simple that this is the listener
        addWindowListener(this);

        //------------------------------------------------------------------
        // create a canvas to draw on
        //------------------------------------------------------------------
        if (null != robotDescFile) {
            m_robot.loadRobot(robotDescFile);
        }
        if (null != fieldDescFile) {
            m_field.loadField(fieldDescFile);
        }
        m_canvas = new PathCanvas(m_graphicsConfig, m_robot, m_field, this, m_pathResourceDir);
        titleChanged();
        add(m_canvas, BorderLayout.CENTER);

        //------------------------------------------------------------------
        // Setup the app menu
        //------------------------------------------------------------------
        try {
            Desktop desktop = Desktop.getDesktop();
            desktop.setQuitHandler((e, r) -> exitPathPlaner());
        } catch (UnsupportedOperationException e) {
            System.out.println("No desktop quit handler setup, not supported by this platform.");
        }

    }


    @Override
    public void titleChanged() {
        setTitle(((null == m_canvas.getPathFile()) ? "Swerve Path Planner" : m_canvas.getPathFile().getName()) +
                " - " + m_field.getTitle());
    }

    static private JMenu createMenu(JMenuBar menubar, String name) {
        final JMenu menuNamed = new JMenu(name);
        menubar.add(menuNamed);
        return menuNamed;
    }

    static private JMenuItem createMenuItem(JMenu menu, String name, ActionListener actionListener) {
        JMenuItem menuItem = new JMenuItem(name);
        menuItem.addActionListener(actionListener);
        menu.add(menuItem);
        return menuItem;
    }

    private void loadGameField() {
        JFileChooser fc = new JFileChooser(m_fieldResourceDir);
        fc.setDialogTitle("Load Field");
        fc.setFileFilter(new FileNameExtensionFilter("JSON file", "json"));
        fc.setAcceptAllFileFilterUsed(false);
        if (JFileChooser.APPROVE_OPTION == fc.showOpenDialog(m_canvas)) {
            File file = fc.getSelectedFile();
            System.out.println("Loading field from: " + file.getAbsolutePath());
            m_field.loadField(file.getAbsolutePath());
            titleChanged();
            m_canvas.resetFieldGeometry();
            m_canvas.repaint();
        } else {
            System.out.println("Load field command cancelled by user.");
        }
    }

    private void loadRobot() {
        JFileChooser fc = new JFileChooser(m_robotResourceDir);
        fc.setDialogTitle("Load Robot");
        fc.setFileFilter(new FileNameExtensionFilter("JSON file", "json"));
        fc.setAcceptAllFileFilterUsed(false);
        if (JFileChooser.APPROVE_OPTION == fc.showOpenDialog(m_canvas)) {
            File file = fc.getSelectedFile();
            System.out.println("Loading robot from: " + file.getAbsolutePath());
            m_robot.loadRobot(file.getAbsolutePath());
            m_canvas.resetRobotGeometry();
            m_canvas.repaint();
        } else {
            System.out.println("Load robot command cancelled by user.");
        }
    }

    private void exitPathPlaner() {
        // All done, dispose of the frame (window)
        if ( m_canvas.modifiedSinceSave()) {
            int result = JOptionPane.showConfirmDialog(this,
                    "There are unsaved changes, do you want to save them", "Save Changes?",
                    JOptionPane.YES_NO_OPTION,
                    JOptionPane.QUESTION_MESSAGE);
            if (result == JOptionPane.YES_OPTION) {
                if (m_canvas.getPathFile() == null) {
                    m_canvas.savePathAs();
                } else {
                    m_canvas.savePath();
                }
            }
        }

        dispose();
    }

    @Override
    public void menuSelected(MenuEvent e) {
        System.out.println(String.format("Menu selected: %s", e.toString()));
        if (e.getSource() == m_menuPath) {
            //enable-disable before showing the menu
            System.out.println("Path Menu enable/disable");
            m_menuPathReloadPath.setEnabled(null != m_canvas.getPathFile());
            m_menuPathSavePath.setEnabled(null != m_canvas.getPathFile());
        }
    }

    @Override
    public void menuDeselected(MenuEvent e) {

    }

    @Override
    public void menuCanceled(MenuEvent e) {

    }
    @Override
    public void actionPerformed(ActionEvent event) {
        final Object src = event.getSource();

        // The 'File' menu
        if (src == m_menuContextLoadField) {
            loadGameField();
        } else if (src == m_menuPathLoadPath) {
            m_canvas.loadPath();
        } else if (src == m_menuPathReloadPath) {
            m_canvas.reloadPath();
        } else if (src == m_menuContextLoadRobot) {
            loadRobot();
        } else if (src == m_menuPathSavePath) {
            m_canvas.savePath();
        } else if (src == m_menuPathSaveAsPath) {
            m_canvas.savePathAs();
         } else if (src == m_menuPathNewPath) {
            m_canvas.newPath();
        } else if (src == m_menuPathClearPath) {
            m_canvas.clearPath();
        } else if (src == m_menuPathPlay) {
            m_canvas.animatePath();
        } else if (src == m_menuSpeedMultiplier) {
            String m = JOptionPane.showInputDialog(this, "Speed Multiplier:",
                    String.format("%.2f", m_canvas.getPath().getSpeedMultiplier()));
            try {
                m_canvas.getPath().setSpeedMultiplier(Utl.clip(Double.parseDouble(m), 0.1, 5.0));
            } catch (NumberFormatException e) {
                JOptionPane.showMessageDialog(this, String.format("'%s' is not a valid number.", m));
            }
        } else if (src == m_menuSwitchAlliance) {
            m_canvas.switchAlliance();
        }

    }

    @Override
    public void windowOpened(WindowEvent e) {

    }

    @Override
    public void windowClosing(WindowEvent e) {
        exitPathPlaner();
    }

    @Override
    public void windowClosed(WindowEvent e) {

    }

    @Override
    public void windowIconified(WindowEvent e) {

    }

    @Override
    public void windowDeiconified(WindowEvent e) {

    }

    @Override
    public void windowActivated(WindowEvent e) {

    }

    @Override
    public void windowDeactivated(WindowEvent e) {

    }

}
