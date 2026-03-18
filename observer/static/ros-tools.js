"use strict"

function parseDataType(typeSpec) {
    const arrayType = typeSpec.startsWith("sequence") || /\[[0-9]*\]$/.test(typeSpec)
    let baseType
    if (typeSpec.startsWith("sequence")) {
        baseType = /sequence<(.*)>/.exec(typeSpec)[1]
    } else if (/\[[0-9]*\]$/.test(typeSpec)) {
        baseType = typeSpec.replace(/\[[0-9]*\]$/, "")
    } else {
        baseType = typeSpec
    }
    const numericalType = baseType.startsWith("int")
        || baseType.startsWith("uint")
        || baseType.startsWith("float")
        || baseType.startsWith("double")
    const numericalLikeType = baseType.startsWith("boolean")
    return {
        array: arrayType,
        numerical: numericalType,
        numericalLike: numericalLikeType || numericalType
    }
}

export { parseDataType }
