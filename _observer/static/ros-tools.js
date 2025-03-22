"use strict"

function parseDataType(typeSpec) {
    const arrayType = typeSpec.startsWith("sequence")
    const baseType = arrayType ? /sequence<(.*)>/.exec(typeSpec)[1] : typeSpec
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
