<?php
$chars = array();
$chars[" "] = array("00000000", "00000000", "00000000", "00000000", "00000000", "00000000", "00000000", "00000000", "00000000", "00000000");
$currentChar = "";
foreach (file("font.in") as $line) {
    $line = rtrim($line, "\n");

    if (isset($line[0]) && $line[0] != " " && $line[0] != "*") {
        $currentChar = $line[0];
        $chars[$currentChar] = array();
        continue;
    }
    
    $line = str_replace(array(" ", "*"), array("0", "1"), $line);
    $chars[$currentChar][] = str_pad("000$line", 8, "0");
}

echo "prog_uchar fontPixel[] PROGMEM = {\n";
$lookup = array();
$index = 0;
foreach ($chars as $k => $char) {
    $lookup[ord($k)] = $index++;
    echo "   ";
    foreach ($char as $ln => $line) {
        if ($ln == 9) {
            break;
        }
        echo " 0x" . str_pad(dechex(bindec($line)), 2, "0", STR_PAD_LEFT);
        if ($ln != 8) {
            echo ",";
        }
    }
    if (count($lookup) != count($chars)) {
        echo " // $k (0x" . dechex(ord($k)) . ")\n";
    } else {
        echo " // $k (0x" . dechex(ord($k)) . ")\n";
    }
}
echo "};\n\n";

echo "unsigned char fontLookup[256] = {";
for ($i = 0; $i < 256; $i++) {
    echo (isset($lookup[$i])) ? $lookup[$i] : 0;
    if ($i < 256) {
        echo ", ";
    }
}
echo "};\n\n";

